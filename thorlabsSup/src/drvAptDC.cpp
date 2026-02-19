/*
 * drvAptDC.cpp
 *
 * DC servo motor axis implementation for Thorlabs APT on Linux.
 * Supports TDC001, KDC101, and similar brushed DC servo controllers.
 *
 * DC controllers use MGMSG_MOT_GET_USTATUSUPDATE (0x0491) for status,
 * which returns position (long), velocity (word), motor current (word),
 * and status bits (dword) in a 14-byte data payload.
 *
 * The poll() override uses a SINGLE USTATUSUPDATE request to get
 * both position and status in one round-trip, avoiding the race
 * condition of two separate requests.
 */

#include <stdio.h>
#include <string.h>

#define epicsExportSharedSymbols

#include "drvApt.h"

AptDCMotorAxis::AptDCMotorAxis(AptController* control, int axisNo,
                                uint8_t channel)
    : AptAxis(control, axisNo, channel)
{
    printf("AptDCMotorAxis: Initializing DC motor axis %d, channel %d "
           "(dest=0x%02X)\n", axisNo, channel, control->getDest());

    /* Enable the channel */
    this->enableChannel();

    /*
     * Read back the velocity parameters stored in the controller.
     * This lets us verify whether they are zero (e.g. from a previous
     * corrupted setVelParams call) and warns the user accordingly.
     */
    {
        uint8_t data[256];
        uint16_t dataLen = 0;
        int ret = requestStatus(MGMSG_MOT_REQ_VELPARAMS,
                                MGMSG_MOT_GET_VELPARAMS,
                                data, sizeof(data), &dataLen);
        if (ret == 0 && dataLen >= sizeof(AptVelParams)) {
            AptVelParams* vp = (AptVelParams*)data;
            printf("AptDCMotorAxis: stored VelParams ch=%u minV=%d "
                   "accel=%d maxV=%d\n",
                   vp->chanIdent, vp->minVelocity,
                   vp->acceleration, vp->maxVelocity);
            if (vp->maxVelocity == 0) {
                printf("AptDCMotorAxis: WARNING - maxVelocity is ZERO! "
                       "Motor will not move. Use Kinesis to restore factory "
                       "velocity params, or use AptSetVelParams() to set them.\n");
            }
        } else {
            printf("AptDCMotorAxis: WARNING - could not read VelParams "
                   "(ret=%d)\n", ret);
        }
    }
}

AptDCMotorAxis::~AptDCMotorAxis()
{
    this->disableChannel();
}

/**
 * Optimized poll — single USTATUSUPDATE gives position + velocity +
 * motor current + status bits in ONE 14-byte response.
 */
asynStatus AptDCMotorAxis::poll(bool* moving)
{
    uint8_t data[256];
    uint16_t dataLen = 0;

    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);

    int32_t pos;
    uint32_t statusBits;

    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        pos = update->position;
        statusBits = update->statusBits;
        lastPosition_ = pos;
        lastStatus_ = statusBits;
        pollOk_ = true;

        APT_INFO("DC poll: pos=%d vel=%u cur=%u status=0x%08X\n",
                 update->position, update->velocity,
                 update->motorCurrent, update->statusBits);
    } else {
        /* Poll failed — use last known values */
        pos = lastPosition_;
        statusBits = lastStatus_;
        APT_ERR("DC poll: USTATUSUPDATE FAILED (ret=%d dataLen=%u), "
                "using cached pos=%d status=0x%08X\n",
                ret, dataLen, pos, statusBits);
    }

    setDoubleParam(this->pC_->motorPosition_, (double)pos);

    if ((statusBits & SB_INMOTIONCW) || (statusBits & SB_INMOTIONCCW)) {
        setIntegerParam(this->pC_->motorStatusMoving_, 1);
        setIntegerParam(this->pC_->motorStatusDirection_,
                        (statusBits & SB_INMOTIONCW) ? 0 : 1);
        setIntegerParam(this->pC_->motorStatusDone_, 0);
        *moving = true;
        APT_INFO("DC poll: MOVING %s\n",
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
        APT_ERR("DC poll: ERROR bit set (0x%08X)\n", statusBits);

    callParamCallbacks();

    return asynSuccess;
}

int AptDCMotorAxis::getPosition()
{
    /*
     * For DC motors, use USTATUSUPDATE which gives position, velocity,
     * motor current, and status bits in one response.
     */
    uint8_t data[256];
    uint16_t dataLen = 0;
    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        APT_TRACE("DC getPosition: pos=%d vel=%u cur=%u\n",
                  update->position, update->velocity, update->motorCurrent);
        return update->position;
    }

    APT_ERR("DC getPosition: USTATUSUPDATE failed (ret=%d), "
            "falling back to POSCOUNTER\n", ret);
    /* Fall back to position counter request */
    return AptAxis::getPosition();
}

uint32_t AptDCMotorAxis::getStatus()
{
    uint8_t data[256];
    uint16_t dataLen = 0;
    int ret = requestStatus(MGMSG_MOT_REQ_USTATUSUPDATE,
                            MGMSG_MOT_GET_USTATUSUPDATE,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptUStatusUpdate)) {
        AptUStatusUpdate* update = (AptUStatusUpdate*)data;
        APT_TRACE("DC getStatus: bits=0x%08X\n", update->statusBits);
        return update->statusBits;
    }

    APT_ERR("DC getStatus: USTATUSUPDATE failed (ret=%d), "
            "falling back to STATUSBITS\n", ret);
    return AptAxis::getStatus();
}
