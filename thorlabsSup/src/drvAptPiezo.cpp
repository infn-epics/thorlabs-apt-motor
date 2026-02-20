/*
 * drvAptPiezo.cpp
 *
 * Piezo axis implementation for Thorlabs APT on Linux.
 * Supports KPZ101, TPZ001, and similar piezo controllers.
 *
 * Piezo controllers use MGMSG_PZ_* messages instead of the standard
 * MGMSG_MOT_* motor messages.  Key differences from DC/stepper:
 *
 *   - Position is a 16-bit value (0–32767) mapping to 0–100% of the
 *     actuator's max travel.  The motor record MRES should be set to
 *     convert between motor record counts and the 0–32767 range.
 *
 *   - No velocity profile: moves are effectively immediate (piezo).
 *     The motor record VELO/VBAS/ACCL fields are not used.
 *
 *   - No homing: piezo actuators do not have limit switches.
 *     The home() method performs a PZ_SET_ZERO instead.
 *
 *   - Closed-loop mode (PZ_POSCONTROL_CLOSEDLOOP) is required for
 *     position feedback.  The driver sets this at initialization.
 *
 *   - Status is read via PZ_GET_PZSTATUSUPDATE, which returns the
 *     output voltage, output position, and status bits in one frame.
 *
 * Position model:
 *   The KPZ101 accepts positions 0–32767, where 32767 = 100% of max travel.
 *   For a 20 µm piezo, MRES = 20.0 / 32767 ≈ 0.000610 µm/count.
 *   The motor record then moves in µm and the driver converts to 0–32767.
 */

#include <stdio.h>
#include <string.h>

#define epicsExportSharedSymbols

#include "drvApt.h"

/* ========================================================================= */
/*  AptPiezoAxis                                                              */
/* ========================================================================= */

AptPiezoAxis::AptPiezoAxis(AptController* control, int axisNo,
                            uint8_t channel)
    : AptAxis(control, axisNo, channel),
      lastPzPosition_(0),
      lastPzStatus_(0),
      controlMode_(PZ_POSCONTROL_CLOSEDLOOP)
{
    printf("AptPiezoAxis: Initializing piezo axis %d, channel %d "
           "(dest=0x%02X)\n", axisNo, channel, control->getDest());

    /* Enable the channel */
    this->enableChannel();

    /*
     * Set closed-loop position control mode.
     * This is required for position readback and position-based moves.
     * In open-loop mode, only voltage control is available.
     */
    this->setPositionControlMode(PZ_POSCONTROL_CLOSEDLOOP);

    /*
     * Read back the max travel to log it for the user.
     */
    {
        uint8_t data[256];
        uint16_t dataLen = 0;
        int ret = requestStatus(MGMSG_PZ_REQ_MAXTRAVEL,
                                MGMSG_PZ_GET_MAXTRAVEL,
                                data, sizeof(data), &dataLen);
        if (ret == 0 && dataLen >= sizeof(AptPzMaxTravel)) {
            AptPzMaxTravel* mt = (AptPzMaxTravel*)data;
            printf("AptPiezoAxis: maxTravel = %u (x100nm = %.1f µm)\n",
                   mt->maxTravel, mt->maxTravel * 0.1);
        } else {
            printf("AptPiezoAxis: WARNING - could not read maxTravel "
                   "(ret=%d)\n", ret);
        }
    }

    /*
     * Read back the max output voltage.
     */
    {
        uint8_t data[256];
        uint16_t dataLen = 0;
        int ret = requestStatus(MGMSG_PZ_REQ_OUTPUTMAXVOLTS,
                                MGMSG_PZ_GET_OUTPUTMAXVOLTS,
                                data, sizeof(data), &dataLen);
        if (ret == 0 && dataLen >= sizeof(AptPzOutputMaxVolts)) {
            AptPzOutputMaxVolts* mv = (AptPzOutputMaxVolts*)data;
            printf("AptPiezoAxis: maxOutputVolts = %.1f V\n",
                   mv->maxVolts * 0.1);
        } else {
            printf("AptPiezoAxis: WARNING - could not read maxOutputVolts "
                   "(ret=%d)\n", ret);
        }
    }
}

AptPiezoAxis::~AptPiezoAxis()
{
    this->disableChannel();
}

/* ---------- Position control mode ---------- */

void AptPiezoAxis::setPositionControlMode(uint8_t mode)
{
    if (!pC_->aptSerial_) return;

    const char* modeStr = "UNKNOWN";
    switch (mode) {
        case PZ_POSCONTROL_OPENLOOP:         modeStr = "Open-Loop"; break;
        case PZ_POSCONTROL_CLOSEDLOOP:       modeStr = "Closed-Loop"; break;
        case PZ_POSCONTROL_OPENLOOP_SMOOTH:  modeStr = "Open-Loop Smooth"; break;
        case PZ_POSCONTROL_CLOSEDLOOP_SMOOTH:modeStr = "Closed-Loop Smooth"; break;
    }

    APT_INFO("setPositionControlMode: setting mode=%d (%s)\n", mode, modeStr);

    pC_->aptLock_.lock();
    aptSendShortMessage(pC_->aptSerial_, MGMSG_PZ_SET_POSCONTROLMODE,
                        channel_, mode,
                        pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    controlMode_ = mode;
    printf("AptPiezoAxis: Control mode set to %s\n", modeStr);
}

/* ---------- Output position ---------- */

void AptPiezoAxis::setOutputPosition(uint16_t pos)
{
    if (!pC_->aptSerial_) return;

    /* Clamp to valid range */
    if (pos > PZ_MAX_POSITION) pos = PZ_MAX_POSITION;

    APT_INFO("setOutputPosition: pos=%u (of %d)\n", pos, PZ_MAX_POSITION);

    AptPzOutputPos params;
    params.chanIdent = channel_;
    params.position = pos;

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_PZ_SET_OUTPUTPOS,
                                 &params, sizeof(params),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("setOutputPosition: send FAILED (ret=%d)\n", ret);
}

uint16_t AptPiezoAxis::getOutputPosition()
{
    uint8_t data[256];
    uint16_t dataLen = 0;

    int ret = requestStatus(MGMSG_PZ_REQ_OUTPUTPOS,
                            MGMSG_PZ_GET_OUTPUTPOS,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptPzOutputPos)) {
        AptPzOutputPos* op = (AptPzOutputPos*)data;
        APT_TRACE("getOutputPosition: pos=%u\n", op->position);
        return op->position;
    }

    APT_ERR("getOutputPosition: FAILED (ret=%d)\n", ret);
    return lastPzPosition_;
}

/* ---------- Output voltage ---------- */

void AptPiezoAxis::setOutputVoltage(int16_t volts)
{
    if (!pC_->aptSerial_) return;

    APT_INFO("setOutputVoltage: volts=%d\n", volts);

    AptPzOutputVolts params;
    params.chanIdent = channel_;
    params.voltage = volts;

    pC_->aptLock_.lock();
    aptSendLongMessage(pC_->aptSerial_, MGMSG_PZ_SET_OUTPUTVOLTS,
                       &params, sizeof(params),
                       pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

/* ---------- asynMotorAxis interface ---------- */

asynStatus AptPiezoAxis::move(double position, int relative,
                               double minVelocity, double maxVelocity,
                               double acceleration)
{
    APT_INFO("piezo move: pos=%.4f rel=%d\n", position, relative);

    /*
     * For piezo, the position value is already in the 0–32767 range
     * (the motor record does the EGU→counts conversion via MRES).
     * We just need to cast to uint16_t and clamp.
     */
    int32_t targetCounts;

    if (relative) {
        targetCounts = (int32_t)lastPzPosition_ + (int32_t)position;
    } else {
        targetCounts = (int32_t)position;
    }

    /* Clamp to 0–32767 range */
    if (targetCounts < 0) targetCounts = 0;
    if (targetCounts > PZ_MAX_POSITION) targetCounts = PZ_MAX_POSITION;

    APT_INFO("piezo move: target=%d of %d\n", targetCounts, PZ_MAX_POSITION);

    /*
     * Piezo moves are near-instantaneous compared to motor poll rate.
     * Still signal motion started to keep the motor record happy.
     */
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    this->setOutputPosition((uint16_t)targetCounts);

    return asynSuccess;
}

asynStatus AptPiezoAxis::home(double minVelocity, double maxVelocity,
                               double acceleration, int forwards)
{
    APT_INFO("piezo home: performing PZ_SET_ZERO\n");

    /*
     * Piezo actuators don't have limit switches.
     * "Homing" means zeroing the current position — the zero point
     * is stored in the controller's EEPROM.
     */
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSendShortMessage(pC_->aptSerial_, MGMSG_PZ_SET_ZERO,
                            channel_, 0x00,
                            pC_->dest_, APT_HOST);
        pC_->aptLock_.unlock();
    }

    /* Wait a moment for the zero to take effect */
    epicsThreadSleep(0.5);

    setIntegerParam(pC_->motorStatusHomed_, 1);
    setIntegerParam(pC_->motorStatusHome_, 1);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus AptPiezoAxis::stop(double acceleration)
{
    APT_INFO("piezo stop: (piezo moves are near-instantaneous, NOP)\n");
    /*
     * Piezo actuators move near-instantaneously; there is no meaningful
     * way to stop mid-move.  Just report done.
     */
    setIntegerParam(pC_->motorStatusDone_, 1);
    setIntegerParam(pC_->motorStatusMoving_, 0);
    callParamCallbacks();
    return asynSuccess;
}

/**
 * Poll the piezo controller for position and status.
 *
 * Uses PZ_GET_PZSTATUSUPDATE which returns output voltage, output position,
 * and status bits in a single response (when in closed-loop mode).
 */
asynStatus AptPiezoAxis::poll(bool* moving)
{
    uint8_t data[256];
    uint16_t dataLen = 0;

    int ret = requestStatus(MGMSG_PZ_REQ_PZSTATUSUPDATE,
                            MGMSG_PZ_GET_PZSTATUSUPDATE,
                            data, sizeof(data), &dataLen);

    uint16_t pos;
    uint32_t statusBits;

    if (ret == 0 && dataLen >= sizeof(AptPzStatusUpdate)) {
        AptPzStatusUpdate* update = (AptPzStatusUpdate*)data;
        pos = update->outputPosition;
        statusBits = update->statusBits;
        lastPzPosition_ = pos;
        lastPzStatus_ = statusBits;
        pollOk_ = true;

        APT_INFO("Piezo poll: outV=%u outPos=%u status=0x%08X\n",
                 update->outputVoltage, update->outputPosition,
                 update->statusBits);
    } else {
        /*
         * PZ_GET_PZSTATUSUPDATE failed — try getting position directly.
         */
        pos = getOutputPosition();
        statusBits = lastPzStatus_;
        APT_ERR("Piezo poll: PZSTATUSUPDATE FAILED (ret=%d dataLen=%u), "
                "using REQ_OUTPUTPOS pos=%u\n", ret, dataLen, pos);
        lastPzPosition_ = pos;
    }

    /* Report position to motor record */
    setDoubleParam(pC_->motorPosition_, (double)pos);

    /*
     * Piezo moves are near-instantaneous (sub-ms response).
     * At the poll rates used by EPICS motor (100ms+), the move
     * is always complete by the time we poll.
     * Report done + not moving.
     */
    setIntegerParam(pC_->motorStatusDone_, 1);
    setIntegerParam(pC_->motorStatusMoving_, 0);
    *moving = false;

    /* No limit switches on piezo; report clear */
    setIntegerParam(pC_->motorStatusLowLimit_, 0);
    setIntegerParam(pC_->motorStatusHighLimit_, 0);

    /* Piezo is always "powered" if connected */
    setIntegerParam(pC_->motorStatusPowerOn_, 1);

    /* Check for connected status */
    if (statusBits & SB_CONNECTED) {
        APT_TRACE("Piezo poll: connected\n");
    }

    callParamCallbacks();

    return asynSuccess;
}
