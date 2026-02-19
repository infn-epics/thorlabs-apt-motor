/*
 * drvAptStepper.cpp
 *
 * Stepper motor axis implementation for Thorlabs APT on Linux.
 * Supports TST001, KST101, BSC10x, BSC20x, LTS, and similar
 * stepper motor controllers.
 *
 * Stepper controllers may use either:
 *   - MGMSG_MOT_GET_STATUSUPDATE (0x0481): position (long),
 *     encoder count (long), status bits (dword) - 14 bytes
 *   - MGMSG_MOT_GET_USTATUSUPDATE (0x0491): position (long),
 *     velocity (word), motor current (word), status bits (dword) - 14 bytes
 *
 * We prefer USTATUSUPDATE for newer controllers, with fallback
 * to STATUSUPDATE for legacy stepper controllers.
 *
 * The poll() override uses a SINGLE status request to get both
 * position and status in one round-trip.
 */

#include <stdio.h>
#include <string.h>

#define epicsExportSharedSymbols

#include "drvApt.h"

AptStepMotorAxis::AptStepMotorAxis(AptController* control, int axisNo,
                                    uint8_t channel)
    : AptAxis(control, axisNo, channel)
{
    printf("AptStepMotorAxis: Initializing stepper motor axis %d, channel %d "
           "(dest=0x%02X)\n", axisNo, channel, control->getDest());

    /* Enable the channel */
    this->enableChannel();
}

AptStepMotorAxis::~AptStepMotorAxis()
{
    this->disableChannel();
}

/**
 * Optimized poll — one request gives both position + status.
 * Try USTATUSUPDATE first, fall back to STATUSUPDATE for legacy.
 */
asynStatus AptStepMotorAxis::poll(bool* moving)
{
    uint8_t data[256];
    uint16_t dataLen = 0;
    int32_t pos;
    uint32_t statusBits;
    bool gotStatus = false;

    /* Try USTATUSUPDATE (newer controllers) */
    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        pos = update->position;
        statusBits = update->statusBits;
        lastPosition_ = pos;
        lastStatus_ = statusBits;
        pollOk_ = true;
        gotStatus = true;
        APT_INFO("Step poll (U): pos=%d vel=%u cur=%u status=0x%08X\n",
                 update->position, update->velocity,
                 update->motorCurrent, update->statusBits);
    }

    /* Fall back to STATUSUPDATE (legacy) */
    if (!gotStatus) {
        ret = requestStatus(MGMSG_MOT_REQ_STATUSUPDATE,
                            MGMSG_MOT_GET_STATUSUPDATE,
                            data, sizeof(data), &dataLen);
        if (ret == 0 && dataLen >= sizeof(AptStatusUpdate)) {
            AptStatusUpdate* update = (AptStatusUpdate*)data;
            pos = update->position;
            statusBits = update->statusBits;
            lastPosition_ = pos;
            lastStatus_ = statusBits;
            pollOk_ = true;
            gotStatus = true;
            APT_INFO("Step poll (S): pos=%d enc=%d status=0x%08X\n",
                     update->position, update->encoderCount, update->statusBits);
        }
    }

    if (!gotStatus) {
        pos = lastPosition_;
        statusBits = lastStatus_;
        APT_ERR("Step poll: both USTATUSUPDATE and STATUSUPDATE FAILED, "
                "using cached pos=%d status=0x%08X\n", pos, statusBits);
    }

    setDoubleParam(this->pC_->motorPosition_, (double)pos);

    if ((statusBits & SB_INMOTIONCW) || (statusBits & SB_INMOTIONCCW)) {
        setIntegerParam(this->pC_->motorStatusMoving_, 1);
        setIntegerParam(this->pC_->motorStatusDirection_,
                        (statusBits & SB_INMOTIONCW) ? 0 : 1);
        setIntegerParam(this->pC_->motorStatusDone_, 0);
        *moving = true;
        APT_INFO("Step poll: MOVING %s\n",
                 (statusBits & SB_INMOTIONCW) ? "CW" : "CCW");
    } else {
        setIntegerParam(this->pC_->motorStatusMoving_, 0);
        setIntegerParam(this->pC_->motorStatusDone_, 1);
        *moving = false;
    }

    setIntegerParam(this->pC_->motorStatusLowLimit_,
                    (statusBits & SB_CWHARDLIMIT) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHighLimit_,
                    (statusBits & SB_CCWHARDLIMIT) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHome_,
                    (statusBits & SB_HOMED) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHomed_,
                    (statusBits & SB_HOMED) ? 1 : 0);

    if (statusBits & SB_ERROR)
        APT_ERR("Step poll: ERROR bit set (0x%08X)\n", statusBits);

    callParamCallbacks();

    return asynSuccess;
}

int AptStepMotorAxis::getPosition()
{
    uint8_t data[256];
    uint16_t dataLen = 0;

    /* Try USTATUSUPDATE */
    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        APT_TRACE("Step getPosition (U): pos=%d\n", update->position);
        return update->position;
    }

    /* Fall back to STATUSUPDATE */
    ret = requestStatus(MGMSG_MOT_REQ_STATUSUPDATE,
                        MGMSG_MOT_GET_STATUSUPDATE,
                        data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptStatusUpdate)) {
        AptStatusUpdate* update = (AptStatusUpdate*)data;
        APT_TRACE("Step getPosition (S): pos=%d\n", update->position);
        return update->position;
    }

    APT_ERR("Step getPosition: both status requests failed\n");
    /* Last resort: position counter */
    return AptAxis::getPosition();
}

uint32_t AptStepMotorAxis::getStatus()
{
    uint8_t data[256];
    uint16_t dataLen = 0;

    /* Try USTATUSUPDATE */
    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        APT_TRACE("Step getStatus (U): bits=0x%08X\n", update->statusBits);
        return update->statusBits;
    }

    /* Fall back to STATUSUPDATE */
    ret = requestStatus(MGMSG_MOT_REQ_STATUSUPDATE,
                        MGMSG_MOT_GET_STATUSUPDATE,
                        data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptStatusUpdate)) {
        AptStatusUpdate* update = (AptStatusUpdate*)data;
        APT_TRACE("Step getStatus (S): bits=0x%08X\n", update->statusBits);
        return update->statusBits;
    }

    APT_ERR("Step getStatus: both status requests failed\n");
    /* Last resort: status bits only */
    return AptAxis::getStatus();
}
