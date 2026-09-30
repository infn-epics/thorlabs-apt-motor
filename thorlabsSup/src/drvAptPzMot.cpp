/*
 * drvAptPzMot.cpp
 *
 * Piezo Inertial Motor (PZMOT) axis implementation for Thorlabs APT
 * on Linux.  Supports KIM101, TIM101, KIM001 controllers.
 *
 * These controllers use the MGMSG_PZMOT_* message family (0x08xx),
 * which is completely separate from the standard MOT_* (0x04xx)
 * and PZ_* (0x06xx) message families.
 *
 * Key protocol details:
 *   - PZMOT_MOVE_ABSOLUTE (0x08D4): 6-byte data, position in steps
 *   - PZMOT_REQ_STATUSUPDATE (0x08E0) → GET_STATUSUPDATE (0x08E1):
 *     56-byte response containing all 4 channels at once
 *   - PZMOT_ACK_STATUSUPDATE (0x08E2): keepalive sent every 10 polls
 *   - Channel ident uses bitmask (1,2,4,8) in data messages
 *   - Stop: MOT_MOVE_STOP (0x0465) — standard message
 *   - Home: MOT_MOVE_HOME (0x0443) — standard message
 *
 * Reference: Thorlabs APT Communications Protocol,
 *            "Messages applicable to TIM101 and KIM101"
 */

#include <stdio.h>
#include <string.h>
#include <cmath>

#define epicsExportSharedSymbols

#include "drvApt.h"

/* ========================================================================= */
/*  AptPzMotAxis                                                              */
/* ========================================================================= */

AptPzMotAxis::AptPzMotAxis(AptController* control, int axisNo,
                            uint8_t channel)
    : AptAxis(control, axisNo, channel),
      chanBitmask_(PZMOT_CHAN_BITMASK(axisNo)),   /* 0→1, 1→2, 2→4, 3→8 */
      chanEnableMode_((uint16_t)(axisNo + 1)),    /* 1,2,3,4 — sequential */
      ackCounter_(0)
{
    printf("AptPzMotAxis: Initializing PZMOT axis %d, channel %d, "
           "bitmask 0x%02X, enableMode=%d (dest=0x%02X)\n",
           axisNo, channel, chanBitmask_, chanEnableMode_,
           control->getDest());

    /* Enable the channel via standard MOD_SET_CHANENABLESTATE */
    this->enableChannel();
}

AptPzMotAxis::~AptPzMotAxis()
{
    this->disableChannel();
}

/* ---- PZMOT-specific operations ---- */

/**
 * Enable this channel on the KIM101 via PZMOT_SET_PARAMS sub-message 0x2B.
 *
 * The KIM101 can only move the currently "selected" channel.
 * Before issuing a move command, we must enable the target channel
 * by sending PZMOT_SET_PARAMS (0x08C0) with sub-message ID 0x2B
 * (KCubeChanEnableMode) and the channel number (1-4).
 *
 * Reference: pyLabLib KinesisDevice._pzmot_enable_channels()
 */
void AptPzMotAxis::pzMotEnableChannel()
{
    if (!pC_->aptSerial_) return;

    APT_INFO("pzMotEnableChannel: enabling channel %d (enableMode=%d)\n",
             channel_, chanEnableMode_);

    /*
     * Build PZMOT_SET_PARAMS data:
     *   bytes 0-1: sub-message ID = 0x002B (KCubeChanEnableMode)
     *   bytes 2-3: channel enable mode (1=Ch1, 2=Ch2, 3=Ch3, 4=Ch4)
     */
    uint8_t data[4];
    data[0] = (uint8_t)(PZMOT_SUBMSG_CHANENABLEMODE & 0xFF);
    data[1] = (uint8_t)((PZMOT_SUBMSG_CHANENABLEMODE >> 8) & 0xFF);
    data[2] = (uint8_t)(chanEnableMode_ & 0xFF);
    data[3] = (uint8_t)((chanEnableMode_ >> 8) & 0xFF);

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_PZMOT_SET_PARAMS,
                                 data, sizeof(data),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("pzMotEnableChannel: FAILED (ret=%d)\n", ret);
    else
        APT_INFO("pzMotEnableChannel: channel %d enabled OK\n", channel_);
}

/**
 * Send PZMOT_MOVE_ABSOLUTE to move to an absolute position in steps.
 */
void AptPzMotAxis::pzMotMoveAbsolute(int32_t position)
{
    if (!pC_->aptSerial_) return;

    APT_INFO("pzMotMoveAbsolute: axis %d, target=%d steps\n",
             axisNo_, position);

    AptPzMotMoveAbs params;
    params.chanIdent = chanBitmask_;
    params.absPosition = position;

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_PZMOT_MOVE_ABSOLUTE,
                                 &params, sizeof(params),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("pzMotMoveAbsolute: FAILED (ret=%d)\n", ret);
    else
        APT_INFO("pzMotMoveAbsolute: command sent OK\n");
}

/**
 * Send PZMOT_MOVE_JOG to jog forward or reverse.
 * Note: Jog uses sequential channel numbering (1-4), NOT bitmask.
 */
void AptPzMotAxis::pzMotMoveJog(int direction)
{
    if (!pC_->aptSerial_) return;

    uint8_t jogChan = (uint8_t)(axisNo_ + 1);  /* 1-based sequential */
    uint8_t jogDir = (direction > 0) ? 0x01 : 0x02;  /* 1=fwd, 2=rev */

    APT_INFO("pzMotMoveJog: axis %d, chan=%d, dir=%d\n",
             axisNo_, jogChan, jogDir);

    pC_->aptLock_.lock();
    aptSendShortMessage(pC_->aptSerial_, MGMSG_PZMOT_MOVE_JOG,
                        jogChan, jogDir,
                        pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

/**
 * Request PZMOT status update — returns all 4 channels at once.
 *
 * Sends PZMOT_REQ_STATUSUPDATE (0x08E0) and waits for
 * PZMOT_GET_STATUSUPDATE (0x08E1) with 56 bytes of data.
 *
 * Also handles the ACK keepalive: after every 10 status updates
 * we send PZMOT_ACK_STATUSUPDATE to keep the controller alive.
 *
 * @return 0 on success, negative on error
 */
int AptPzMotAxis::pzMotRequestStatus(AptPzMotStatusUpdate* status)
{
    if (!pC_->aptSerial_) return -1;

    pC_->aptLock_.lock();

    /* Send the status request — header-only, channel=0 for all channels */
    APT_TRACE("pzMotRequestStatus: sending REQ 0x%04X\n",
              MGMSG_PZMOT_REQ_STATUSUPDATE);

    int ret = aptSendShortMessage(pC_->aptSerial_,
                                  MGMSG_PZMOT_REQ_STATUSUPDATE,
                                  0x01, 0x00,
                                  pC_->dest_, APT_HOST);
    if (ret != 0) {
        pC_->aptLock_.unlock();
        return -1;
    }

    /* Read responses until we get PZMOT_GET_STATUSUPDATE */
    uint8_t header[6];
    uint8_t data[256];
    uint16_t dataLen = 0;
    int attempts = 0;
    const int maxAttempts = 20;

    while (attempts < maxAttempts) {
        ret = aptReceiveMessage(pC_->aptSerial_, header,
                                data, sizeof(data), &dataLen, 2000);
        if (ret != 0) {
            pC_->aptLock_.unlock();
            return ret;
        }

        uint16_t msgId = aptGetMsgId(header);
        APT_TRACE("pzMotRequestStatus: received msg 0x%04X, dataLen=%u\n",
                  msgId, dataLen);

        if (msgId == MGMSG_PZMOT_GET_STATUSUPDATE &&
            dataLen >= sizeof(AptPzMotStatusUpdate)) {
            memcpy(status, data, sizeof(AptPzMotStatusUpdate));
            pC_->aptLock_.unlock();

            /* Send ACK keepalive every 10 status updates */
            ackCounter_++;
            if (ackCounter_ >= 10) {
                ackCounter_ = 0;
                pC_->aptLock_.lock();
                aptSendShortMessage(pC_->aptSerial_,
                                    MGMSG_PZMOT_ACK_STATUSUPDATE,
                                    0x00, 0x00,
                                    pC_->dest_, APT_HOST);
                pC_->aptLock_.unlock();
                APT_TRACE("pzMotRequestStatus: sent ACK keepalive\n");
            }

            return 0;
        }

        /* Skip known spontaneous messages */
        if (msgId == MGMSG_PZMOT_MOVE_COMPLETED ||
            msgId == MGMSG_MOT_MOVE_COMPLETED ||
            msgId == MGMSG_MOT_MOVE_STOPPED ||
            msgId == MGMSG_MOT_MOVE_HOMED ||
            msgId == MGMSG_MOT_GET_USTATUSUPDATE ||
            msgId == MGMSG_MOT_GET_STATUSUPDATE ||
            msgId == MGMSG_PZ_GET_PZSTATUSUPDATE) {
            APT_TRACE("pzMotRequestStatus: skipping spontaneous 0x%04X\n",
                      msgId);
            attempts++;
            continue;
        }

        /* Controller error response */
        if (msgId == MGMSG_HW_RESPONSE || msgId == MGMSG_HW_RICHRESPONSE) {
            APT_ERR("pzMotRequestStatus: HW error response 0x%04X\n", msgId);
            pC_->aptLock_.unlock();
            return -3;
        }

        APT_ERR("pzMotRequestStatus: unexpected msg 0x%04X, attempt %d\n",
                msgId, attempts);
        attempts++;
    }

    pC_->aptLock_.unlock();
    APT_ERR("pzMotRequestStatus: TIMEOUT after %d attempts\n", maxAttempts);
    return -2;
}

int AptPzMotAxis::pzMotGetDriveParams(
    AptPzMotDriveOpParams* params)
{
    if (!pC_->aptSerial_ || !params)
        return -1;

    pC_->aptLock_.lock();

    /*
     * For PZMOT_REQ_PARAMS:
     * param1 = sub-message ID
     * param2 = channel bitmask
     */
    int ret = aptSendShortMessage(
        pC_->aptSerial_,
        MGMSG_PZMOT_REQ_PARAMS,
        (uint8_t)PZMOT_SUBMSG_DRIVEOPPARAMS,
        (uint8_t)chanBitmask_,
        pC_->dest_,
        APT_HOST);

    if (ret != 0) {
        pC_->aptLock_.unlock();
        APT_ERR("PZMOT: failed to request DriveOP params "
                "for axis %d, ret=%d\n",
                axisNo_, ret);
        return ret;
    }

    uint8_t header[6];
    uint8_t data[256];
    uint16_t dataLen = 0;

    for (int attempt = 0; attempt < 20; attempt++) {
        ret = aptReceiveMessage(
            pC_->aptSerial_,
            header,
            data,
            sizeof(data),
            &dataLen,
            2000);

        if (ret != 0) {
            pC_->aptLock_.unlock();
            APT_ERR("PZMOT: timeout/error reading DriveOP params "
                    "for axis %d, ret=%d\n",
                    axisNo_, ret);
            return ret;
        }

        uint16_t msgId = aptGetMsgId(header);

        if (msgId == MGMSG_PZMOT_GET_PARAMS &&
            dataLen >= sizeof(AptPzMotDriveOpParams)) {

            AptPzMotDriveOpParams received;
            memcpy(&received, data, sizeof(received));

            /*
             * Ignore a GET_PARAMS response belonging to another
             * sub-message or another channel.
             * The spec uses bitmask encoding (1,2,4,8), but some
             * firmware versions answer with the channel number (1-4),
             * as already handled in poll(). Accept both.
             */
            if (received.subMsgId !=
                    PZMOT_SUBMSG_DRIVEOPPARAMS ||
                (received.chanIdent != chanBitmask_ &&
                 received.chanIdent != (uint16_t)(axisNo_ + 1))) {
                continue;
            }

            *params = received;
            pC_->aptLock_.unlock();

            APT_INFO(
                "PZMOT DriveOP read: axis=%d chan=0x%X "
                "voltage=%u rate=%u accel=%u\n",
                axisNo_,
                received.chanIdent,
                received.maxVoltage,
                received.stepRate,
                received.stepAcceleration);

            return 0;
        }

        /*
         * These messages can already be present in the receive
         * buffer. They do not belong to this request.
         */
        if (msgId == MGMSG_PZMOT_GET_STATUSUPDATE ||
            msgId == MGMSG_PZMOT_MOVE_COMPLETED ||
            msgId == MGMSG_MOT_MOVE_COMPLETED ||
            msgId == MGMSG_MOT_MOVE_STOPPED) {
            continue;
        }

        if (msgId == MGMSG_HW_RESPONSE ||
            msgId == MGMSG_HW_RICHRESPONSE) {
            pC_->aptLock_.unlock();
            APT_ERR("PZMOT: controller rejected DriveOP request "
                    "for axis %d\n",
                    axisNo_);
            return -3;
        }
    }

    pC_->aptLock_.unlock();

    APT_ERR("PZMOT: no matching DriveOP response "
            "for axis %d\n",
            axisNo_);

    return -2;
}

int AptPzMotAxis::pzMotSetDriveParams(
    double maxVelocity,
    double acceleration)
{
    if (!pC_->aptSerial_)
        return -1;

    if (!std::isfinite(maxVelocity) ||
        !std::isfinite(acceleration)) {
        APT_ERR("PZMOT: non-finite velocity/acceleration "
                "for axis %d\n",
                axisNo_);
        return -1;
    }

    uint32_t stepRate =
        (uint32_t)std::llround(std::fabs(maxVelocity));

    uint32_t stepAcceleration =
        (uint32_t)std::llround(std::fabs(acceleration));

    /*
     * KIM101 limits documented by Thorlabs.
     * Reject invalid values instead of silently changing them.
     */
    if (stepRate < 1 || stepRate > 2000) {
        APT_ERR("PZMOT: requested velocity %u is outside "
                "the valid range 1..2000 steps/s\n",
                stepRate);
        return -1;
    }

    if (stepAcceleration < 1 ||
        stepAcceleration > 100000) {
        APT_ERR("PZMOT: requested acceleration %u is outside "
                "the valid range 1..100000 steps/s^2\n",
                stepAcceleration);
        return -1;
    }

    /*
     * Read the current structure so that the existing piezo
     * voltage is preserved.
     */
    AptPzMotDriveOpParams params;

    int ret = pzMotGetDriveParams(&params);
    if (ret != 0)
        return ret;

    /*
     * Refuse to overwrite a suspicious voltage value.
     * Do not invent a default voltage.
     */
    if (params.maxVoltage < 85 ||
        params.maxVoltage > 125) {
        APT_ERR("PZMOT: controller returned invalid/suspicious "
                "maxVoltage=%u for axis %d; profile not changed\n",
                params.maxVoltage, axisNo_);
        return -1;
    }

    params.subMsgId =
        PZMOT_SUBMSG_DRIVEOPPARAMS;
    params.chanIdent = chanBitmask_;
    params.stepRate = stepRate;
    params.stepAcceleration = stepAcceleration;

    pC_->aptLock_.lock();

    ret = aptSendLongMessage(
        pC_->aptSerial_,
        MGMSG_PZMOT_SET_PARAMS,
        &params,
        sizeof(params),
        pC_->dest_,
        APT_HOST);

    pC_->aptLock_.unlock();

    if (ret != 0) {
        APT_ERR("PZMOT: failed to set DriveOP params "
                "for axis %d, ret=%d\n",
                axisNo_, ret);
        return ret;
    }

    /*
     * Read back and verify. A successful socket write alone does
     * not prove that the controller accepted the parameters.
     */
    AptPzMotDriveOpParams verify;

    ret = pzMotGetDriveParams(&verify);
    if (ret != 0)
        return ret;

    if (verify.maxVoltage != params.maxVoltage ||
        verify.stepRate != params.stepRate ||
        verify.stepAcceleration !=
            params.stepAcceleration) {

        APT_ERR(
            "PZMOT: DriveOP verification failed for axis %d: "
            "requested V=%u rate=%u accel=%u, "
            "read V=%u rate=%u accel=%u\n",
            axisNo_,
            params.maxVoltage,
            params.stepRate,
            params.stepAcceleration,
            verify.maxVoltage,
            verify.stepRate,
            verify.stepAcceleration);

        return -1;
    }

    APT_INFO(
        "PZMOT DriveOP set: axis=%d voltage=%u "
        "rate=%u accel=%u\n",
        axisNo_,
        params.maxVoltage,
        params.stepRate,
        params.stepAcceleration);

    return 0;
}

/* ---- asynMotorAxis interface ---- */

asynStatus AptPzMotAxis::move(double position, int relative,
                               double minVelocity, double maxVelocity,
                               double acceleration)
{
    APT_INFO("PZMOT move: pos=%.4f rel=%d minVel=%.4f maxVel=%.4f accel=%.4f \n", position, relative, minVelocity, maxVelocity, acceleration);


    /* Flush stale messages */
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSerialFlush(pC_->aptSerial_);
        pC_->aptLock_.unlock();
    }

    /*
     * Enable this channel on the KIM101 before moving.
     * The KIM101 can only move the currently "selected" channel.
     * Without this, only the channel selected on the front panel moves.
     */
    this->pzMotEnableChannel();

    /*
     * Program this move's requested EPICS velocity and
     * acceleration while preserving maxVoltage.
     * A failure here must not block the move: the controller
     * keeps its previous drive parameters, so warn and go on.
     */
    if (this->pzMotSetDriveParams(
            maxVelocity, acceleration) != 0) {
        fprintf(stderr, "drvApt: PZMOT move: could not configure "
                "velocity profile for axis %d (vel=%.1f accel=%.1f); "
                "moving with controller's current drive parameters\n",
                axisNo_, maxVelocity, acceleration);
    }

    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    if (relative) {
        /* For relative moves, read current position and compute absolute */
        int32_t currentPos = lastPosition_;
        int32_t target = currentPos + (int32_t)position;
        APT_INFO("PZMOT move: RELATIVE %d steps (cur=%d, target=%d)\n",
                 (int32_t)position, currentPos, target);
        this->pzMotMoveAbsolute(target);
    } else {
        APT_INFO("PZMOT move: ABSOLUTE to %d steps\n", (int32_t)position);
        this->pzMotMoveAbsolute((int32_t)position);
    }

    return asynSuccess;
}

asynStatus AptPzMotAxis::poll(bool* moving)
{
    AptPzMotStatusUpdate statusAll;
    int ret = pzMotRequestStatus(&statusAll);

    int32_t pos = lastPosition_;
    uint32_t statusBits = lastStatus_;

    if (ret == 0) {
        /*
         * Find our channel in the 4-channel response.
         * Channels use bitmask encoding: 1, 2, 4, 8.
         */
        bool found = false;
        for (int i = 0; i < 4; i++) {
            if (statusAll.channel[i].chanIdent == chanBitmask_) {
                pos = statusAll.channel[i].position;
                statusBits = statusAll.channel[i].statusBits;
                found = true;
                APT_INFO("PZMOT poll: axis %d (bitmask 0x%02X): "
                         "pos=%d status=0x%08X\n",
                         axisNo_, chanBitmask_, pos, statusBits);
                break;
            }
        }

        if (!found) {
            /*
             * Channel bitmask not found in response — try by index.
             * Some firmware versions may return channels in order (0-3).
             */
            if (axisNo_ < 4) {
                pos = statusAll.channel[axisNo_].position;
                statusBits = statusAll.channel[axisNo_].statusBits;
                APT_INFO("PZMOT poll: axis %d (by index): "
                         "pos=%d status=0x%08X chanId=%d\n",
                         axisNo_, pos, statusBits,
                         statusAll.channel[axisNo_].chanIdent);
                found = true;
            }
        }

        if (found) {
            lastPosition_ = pos;
            lastStatus_ = statusBits;
            pollOk_ = true;
        } else {
            APT_ERR("PZMOT poll: channel bitmask 0x%02X not found\n",
                    chanBitmask_);
        }
    } else {
        APT_ERR("PZMOT poll: status request FAILED (ret=%d), "
                "using cached pos=%d status=0x%08X\n",
                ret, pos, statusBits);
    }

    /* Update EPICS motor record parameters */
    setDoubleParam(this->pC_->motorPosition_, (double)pos);

    /* Check motion bits — same as MOT_* status bits */
    bool isMoving = (statusBits & SB_INMOTIONCW) ||
                    (statusBits & SB_INMOTIONCCW) ||
                    (statusBits & SB_JOGGINGCW) ||
                    (statusBits & SB_JOGGINGCCW) ||
                    (statusBits & SB_HOMING);

    if (isMoving) {
        setIntegerParam(this->pC_->motorStatusMoving_, 1);
        setIntegerParam(this->pC_->motorStatusDirection_,
                        (statusBits & SB_INMOTIONCW) ? 0 : 1);
        setIntegerParam(this->pC_->motorStatusDone_, 0);
        *moving = true;
        APT_INFO("PZMOT poll: MOVING\n");
    } else {
        setIntegerParam(this->pC_->motorStatusMoving_, 0);
        setIntegerParam(this->pC_->motorStatusDone_, 1);
        *moving = false;
    }

    /* Limit switches */
    setIntegerParam(this->pC_->motorStatusLowLimit_,
                    (statusBits & SB_CWHARDLIMIT) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHighLimit_,
                    (statusBits & SB_CCWHARDLIMIT) ? 1 : 0);

    /* Homed status */
    setIntegerParam(this->pC_->motorStatusHome_,
                    (statusBits & SB_HOMED) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHomed_,
                    (statusBits & SB_HOMED) ? 1 : 0);

    /* Connected / enabled */
    if (statusBits & SB_ENABLED)
        APT_TRACE("PZMOT poll: channel enabled\n");
    if (statusBits & SB_CONNECTED)
        APT_TRACE("PZMOT poll: motor connected\n");

    /* Error reporting */
    if (statusBits & SB_ERROR)
        APT_ERR("PZMOT poll: ERROR bit set (0x%08X)\n", statusBits);
    if (statusBits & SB_OVERCURRENT)
        APT_ERR("PZMOT poll: excessive current\n");
    if (statusBits & SB_OVERTEMP)
        APT_ERR("PZMOT poll: excessive temperature\n");

    callParamCallbacks();

    return asynSuccess;
}

asynStatus AptPzMotAxis::stop(double acceleration)
{
    APT_INFO("PZMOT stop: axis %d\n", axisNo_);

    /* Flush stale messages */
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSerialFlush(pC_->aptSerial_);
        pC_->aptLock_.unlock();
    }

    /*
     * Stop by issuing a relative move of 0 steps (pyLabLib approach).
     * This is more reliable than MOT_MOVE_STOP for piezo inertial motors.
     */
    this->pzMotEnableChannel();
    this->pzMotMoveAbsolute(lastPosition_);
    return asynSuccess;
}

asynStatus AptPzMotAxis::home(double minVelocity, double maxVelocity,
                               double acceleration, int forwards)
{
    APT_INFO("PZMOT home: axis %d — zeroing position counter\n", axisNo_);

    /*
     * KIM101 piezo inertial motors do NOT support traditional homing
     * (no home switch).  Instead, "home" zeroes the position counter
     * at the current physical position via PZMOT_SET_PARAMS sub-message
     * 0x40 (CURRENT_POS).
     */

    uint8_t data[12];
    /* sub-message ID = 0x0040 (CURRENT_POS) */
    data[0] = (uint8_t)(PZMOT_SUBMSG_CURRENT_POS & 0xFF);
    data[1] = (uint8_t)((PZMOT_SUBMSG_CURRENT_POS >> 8) & 0xFF);
    /* channel bitmask */
    data[2] = (uint8_t)(chanBitmask_ & 0xFF);
    data[3] = (uint8_t)((chanBitmask_ >> 8) & 0xFF);
    /* reserved (4 bytes) */
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
    /* position = 0 (4 bytes, little-endian) */
    data[8] = 0; data[9] = 0; data[10] = 0; data[11] = 0;

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_PZMOT_SET_PARAMS,
                                 data, sizeof(data),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("PZMOT home: zero position FAILED (ret=%d)\n", ret);
    else
        APT_INFO("PZMOT home: position counter zeroed OK\n");

    lastPosition_ = 0;

    /* Mark done immediately — this is not a move */
    setDoubleParam(this->pC_->motorPosition_, 0.0);
    setIntegerParam(pC_->motorStatusDone_, 1);
    setIntegerParam(pC_->motorStatusMoving_, 0);
    setIntegerParam(pC_->motorStatusHomed_, 1);
    setIntegerParam(pC_->motorStatusHome_, 1);
    callParamCallbacks();

    return asynSuccess;
}
