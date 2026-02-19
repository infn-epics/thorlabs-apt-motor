/*
 * drvApt.cpp
 *
 * EPICS asynMotor driver for Thorlabs APT controllers on Linux.
 * Main controller and base axis implementation using APT protocol
 * over USB-to-serial (FTDI) or TCP (ser2net for remote access).
 *
 * This replaces the Windows-only Kinesis library with direct
 * APT binary protocol communication.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>

#include "drvApt.h"

/* Global debug level — settable from IOC shell: var drvAptDebug 3 */
int drvAptDebug = 0;
extern "C" { epicsExportAddress(int, drvAptDebug); }

/* ========================================================================= */
/*  AptController                                                             */
/* ========================================================================= */

AptController::AptController(const char* asyn_port, const char* devicePath,
                             int type, int numAxes,
                             double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(asyn_port,
                          (numAxes > 1) ? numAxes :
                          (numAxes == 0) ? APT_MAX_AXES : 1,
                          0,     /* numParams */
                          0,     /* interfaceMask */
                          0,     /* interruptMask */
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,     /* autoConnect */
                          0, 0), /* priority, stackSize */
      aptSerial_(NULL),
      dest_(APT_GENERIC_USB),
      numAxes_(0),
      motorType_(type)
{
    strncpy(devicePath_, devicePath, sizeof(devicePath_) - 1);
    devicePath_[sizeof(devicePath_) - 1] = '\0';

    APT_ERR("AptController: opening device %s\n", devicePath);

    /* Open the serial connection */
    aptSerial_ = aptSerialOpen(devicePath);
    if (!aptSerial_) {
        printf("AptController: ERROR - Cannot open device %s\n", devicePath);
        return;
    }

    /*
     * Drain any stale data left in the ser2net TCP buffer from a previous
     * session (e.g. after Ctrl-C).  A single flush() call only empties data
     * already in the OS socket buffer.  We also do a short read-drain loop
     * with a brief timeout so any frames that arrive in the first 200 ms
     * (spontaneous status updates still in flight) are discarded before we
     * send HW_REQ_INFO.
     */
    aptSerialFlush(aptSerial_);
    {
        uint8_t drainHdr[6];
        uint8_t drainData[512];
        uint16_t drainLen = 0;
        int drainCount = 0;
        /* drain up to 32 messages with 100 ms timeout each */
        while (drainCount < 32) {
            int dr = aptReceiveMessage(aptSerial_, drainHdr, drainData,
                                       sizeof(drainData), &drainLen, 100);
            if (dr == -2) break;   /* timeout — buffer is empty */
            if (dr != 0) break;    /* error */
            drainCount++;
        }
        if (drainCount > 0)
            printf("AptController: drained %d stale messages from "
                   "ser2net buffer\n", drainCount);
    }

    /*
     * Send HW_REQ_INFO to identify the controller.
     * This also verifies the connection is working.
     */
    aptSendShortMessage(aptSerial_, MGMSG_HW_REQ_INFO,
                        0x00, 0x00, dest_, APT_HOST);

    uint8_t header[6];
    uint8_t data[256];
    uint16_t dataLen = 0;
    int hwNumChannels = 1;

    int ret = aptReceiveMessage(aptSerial_, header, data, sizeof(data),
                                &dataLen, 3000);
    if (ret == 0 && aptGetMsgId(header) == MGMSG_HW_GET_INFO && dataLen >= 84) {
        AptHwInfo* info = (AptHwInfo*)data;
        char model[9];
        memcpy(model, info->modelNumber, 8);
        model[8] = '\0';
        hwNumChannels = info->numChannels;
        printf("AptController: Connected to %s (S/N: %u, FW: %d.%d.%d, "
               "HW: %d, Channels: %d)\n",
               model, info->serialNumber,
               info->firmwareMajor, info->firmwareInterim, info->firmwareMinor,
               info->hwVersion, hwNumChannels);
    } else {
        printf("AptController: WARNING - No response to HW_REQ_INFO "
               "(ret=%d, dataLen=%u). Device may not be ready.\n",
               ret, dataLen);
    }

    /*
     * Determine actual number of axes to create.
     *   numAxes=0: auto-detect from HW_GET_INFO response
     *   numAxes=1: single-channel USB (backward compatible)
     *   numAxes>1: explicit multi-channel count
     */
    int actualAxes;
    if (numAxes == 0) {
        actualAxes = (hwNumChannels > 0) ? hwNumChannels : 1;
        printf("AptController: auto-detected %d channel(s)\n", actualAxes);
    } else {
        actualAxes = numAxes;
    }

    /*
     * Set destination byte based on channel count.
     * Per the APT protocol specification:
     *   - Single-channel USB units (TDC001, KDC101, KST101): dest = 0x50
     *   - Multi-channel bay/rack systems (BSC102, BSC103, BBD10x):
     *     dest = 0x11 (motherboard) and ChanIdent selects the channel
     */
    if (actualAxes > 1) {
        dest_ = APT_RACK;
        printf("AptController: multi-channel mode — dest=0x%02X (rack)\n",
               dest_);

        /*
         * For bay-type systems, enumerate which bays are populated.
         * This is informational; we create axes based on actualAxes.
         */
        for (int bay = 0; bay < actualAxes; bay++) {
            aptSendShortMessage(aptSerial_, MGMSG_RACK_REQ_BAYUSED,
                                (uint8_t)bay, 0x00, APT_RACK, APT_HOST);
            uint8_t bayHdr[6];
            uint8_t bayData[64];
            uint16_t bayLen = 0;
            ret = aptReceiveMessage(aptSerial_, bayHdr, bayData,
                                    sizeof(bayData), &bayLen, 1000);
            if (ret == 0 && aptGetMsgId(bayHdr) == MGMSG_RACK_GET_BAYUSED) {
                /*
                 * RACK_GET_BAYUSED: byte 2 = bay number, byte 3 = state
                 *   state: 0x01 = bay occupied, 0x02 = bay empty
                 */
                uint8_t bayState = bayHdr[3];
                printf("AptController: Bay %d %s\n", bay,
                       (bayState == 0x01) ? "OCCUPIED" :
                       (bayState == 0x02) ? "EMPTY" : "UNKNOWN");
            } else {
                /* Controller may not support RACK_REQ_BAYUSED (e.g. older FW) */
                APT_INFO("AptController: Bay %d enumeration not supported "
                         "(ret=%d)\n", bay, ret);
                break;  /* No point trying remaining bays */
            }
        }
    } else {
        dest_ = APT_GENERIC_USB;
        APT_INFO("AptController: single-channel mode — dest=0x%02X\n", dest_);
    }

    /*
     * Create axis objects.
     * Each axis gets:
     *   - axisNo  = 0-based index for EPICS motor record ADDR field
     *   - chanId  = 1-based APT channel identifier (APT_CHANNEL_1, _2, ...)
     */
    numAxes_ = actualAxes;
    for (int i = 0; i < actualAxes; i++) {
        uint8_t chanId = (uint8_t)(i + 1);  /* APT channels are 1-based */
        if (type == APT_DC_MOTOR) {
            new AptDCMotorAxis(this, i, chanId);
        } else if (type == APT_STEP_MOTOR) {
            new AptStepMotorAxis(this, i, chanId);
        }
    }

    /*
     * Do NOT call MGMSG_HW_START_UPDATEMSGS here.
     *
     * That command makes the controller send unsolicited USTATUSUPDATE
     * messages every ~100ms into the TCP buffer (via ser2net).  These
     * spontaneous messages cause two problems:
     *
     *  1. After sending MOVE_ABSOLUTE, the TCP buffer still contains
     *     pre-move status messages with the old position.  The next
     *     poll() reads one of those stale messages, sees no motion, and
     *     the motor record immediately retries the move — restarting the
     *     controller from the current position every 200 ms so the stage
     *     never reaches the target.
     *
     *  2. Unsolicited messages of the same ID as requested ones make it
     *     impossible to match request/response pairs reliably.
     *
     * We rely entirely on explicit REQ/GET polling in poll(), which is
     * sufficient and race-free.
     */

    /* Start the EPICS motor poller */
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

AptController::~AptController()
{
    if (aptSerial_) {
        /* Stop status updates and disconnect gracefully */
        aptSendShortMessage(aptSerial_, MGMSG_HW_STOP_UPDATEMSGS,
                            0x00, 0x00, dest_, APT_HOST);
        usleep(100000);
        aptSendShortMessage(aptSerial_, MGMSG_HW_DISCONNECT,
                            0x00, 0x00, dest_, APT_HOST);
        usleep(100000);
        aptSerialClose(aptSerial_);
        aptSerial_ = NULL;
    }
}

AptAxis* AptController::getAxis(asynUser* pasynUser)
{
    return static_cast<AptAxis*>(asynMotorController::getAxis(pasynUser));
}

AptAxis* AptController::getAxis(int axis)
{
    return static_cast<AptAxis*>(asynMotorController::getAxis(axis));
}

/* ========================================================================= */
/*  AptAxis - Base class                                                      */
/* ========================================================================= */

AptAxis::AptAxis(AptController* pc, int axisNo, uint8_t channel)
    : asynMotorAxis(pc, axisNo),
      pC_(pc),
      channel_(channel),
      lastPosition_(0),
      lastStatus_(0),
      pollOk_(false)
{
    setDoubleParam(pC_->motorEncoderPosition_, 0.0);
    callParamCallbacks();
}

int AptAxis::requestStatus(uint16_t reqMsgId, uint16_t getMsgId,
                           void* data, uint16_t dataMaxLen, uint16_t* dataLen)
{
    if (!pC_->aptSerial_) return -1;

    pC_->aptLock_.lock();

    /* Send the request */
    APT_TRACE("requestStatus: sending REQ 0x%04X, expecting GET 0x%04X (ch=%d)\n",
              reqMsgId, getMsgId, channel_);

    int ret = aptSendShortMessage(pC_->aptSerial_, reqMsgId,
                                  channel_, 0x00,
                                  pC_->dest_, APT_HOST);
    if (ret != 0) {
        pC_->aptLock_.unlock();
        return -1;
    }

    /*
     * Read responses until we get the expected message or timeout.
     * We may receive status update messages in between.
     */
    uint8_t header[6];
    int attempts = 0;
    const int maxAttempts = 20;

    while (attempts < maxAttempts) {
        ret = aptReceiveMessage(pC_->aptSerial_, header,
                                data, dataMaxLen, dataLen, 2000);
        if (ret != 0) {
            pC_->aptLock_.unlock();
            return ret;
        }

        uint16_t msgId = aptGetMsgId(header);
        APT_TRACE("requestStatus: received msg 0x%04X, dataLen=%u\n",
                  msgId, *dataLen);
        if (msgId == getMsgId) {
            APT_TRACE("requestStatus: GOT expected 0x%04X, dataLen=%u\n",
                      getMsgId, *dataLen);
            pC_->aptLock_.unlock();
            return 0;
        }

        /* Handle spontaneous status update messages */
        if (msgId == MGMSG_MOT_GET_USTATUSUPDATE ||
            msgId == MGMSG_MOT_GET_STATUSUPDATE ||
            msgId == MGMSG_MOT_MOVE_COMPLETED ||
            msgId == MGMSG_MOT_MOVE_STOPPED ||
            msgId == MGMSG_MOT_MOVE_HOMED) {
            /* These are valid spontaneous messages; continue waiting */
            APT_TRACE("requestStatus: skipping spontaneous msg 0x%04X\n", msgId);
            attempts++;
            continue;
        }

        /* Unknown message; log and continue */
        APT_ERR("requestStatus: unexpected msg 0x%04X "
                "(waiting for 0x%04X), attempt %d\n", msgId, getMsgId, attempts);
        attempts++;
    }

    pC_->aptLock_.unlock();
    APT_ERR("requestStatus: TIMEOUT waiting for 0x%04X after %d attempts\n",
            getMsgId, maxAttempts);
    return -2;
}

asynStatus AptAxis::move(double position, int relative,
                          double minVelocity, double maxVelocity,
                          double acceleration)
{
    APT_INFO("move: pos=%.4f rel=%d minV=%.4f maxV=%.4f accel=%.4f\n",
             position, relative, minVelocity, maxVelocity, acceleration);
    APT_INFO("move: encoder counts = %d (int32)\n", (int32_t)position);

    /*
     * Signal to the motor record that a move has started BEFORE sending
     * the command.  Without this, poll() runs immediately after move()
     * returns, reads the old (pre-move) status, sees DONE=1, and the
     * motor record retries the move every poll cycle — continuously
     * restarting the controller so the stage never reaches the target.
     */
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    /*
     * Flush any stale spontaneous messages that may have accumulated in
     * the TCP/serial receive buffer (e.g. leftover USTATUSUPDATE frames
     * sent before this move command).  Without this, the next poll()
     * could read a pre-move status and incorrectly report no motion.
     * Must hold aptLock_ while flushing to avoid racing with the poller.
     */
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSerialFlush(pC_->aptSerial_);
        pC_->aptLock_.unlock();
    }

    /*
     * DO NOT call setVelParams() here.
     *
     * The motor record provides velocity in steps/sec from the VELO field,
     * but the KDC101 APT protocol expects internally-scaled units that differ
     * by a large factor (Kvm ≈ 42941.66 for velocity).  Sending unscaled
     * values overwrites the controller's velocity profile with a near-zero
     * speed (~1 count/s ≈ 30 nm/s), making the move practically invisible.
     *
     * Use the factory/EEPROM velocity profile stored in the controller.
     * To change velocity, use Kinesis or the AptSetVelParams IOC command.
     */

    if (relative) {
        APT_INFO("move: RELATIVE %d counts\n", (int32_t)position);
        this->moveRelative((int32_t)position);
    } else {
        APT_INFO("move: ABSOLUTE to %d counts\n", (int32_t)position);
        this->moveToPosition((int32_t)position);
    }

    return asynSuccess;
}

asynStatus AptAxis::home(double minVelocity, double maxVelocity,
                          double acceleration, int forwards)
{
    APT_INFO("home: forwards=%d\n", forwards);

    /* Tell the motor record a motion is starting immediately, before sending
     * the command.  Without this the poller runs at the idle rate and the
     * record doesn't snapshot the start position for homed-state tracking. */
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    /* Flush stale messages so the first post-home poll reads fresh data */
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSerialFlush(pC_->aptSerial_);
        pC_->aptLock_.unlock();
    }

    this->doHome();
    return asynSuccess;
}

asynStatus AptAxis::poll(bool* moving)
{
    int pos = this->getPosition();
    uint32_t status = this->getStatus();

    /* If poll succeeded, cache values; otherwise use last known */
    if (pos != 0 || pollOk_ == false) {
        lastPosition_ = pos;
        pollOk_ = true;
    }
    lastStatus_ = status;

    APT_INFO("poll: position=%d (%.6f mm), statusBits=0x%08X\n",
             lastPosition_,
             (double)lastPosition_ * 0.000028939405515,  /* KDC101 typical MRES */
             lastStatus_);

    setDoubleParam(this->pC_->motorPosition_, (double)lastPosition_);

    if ((lastStatus_ & SB_INMOTIONCW) || (lastStatus_ & SB_INMOTIONCCW)) {
        setIntegerParam(this->pC_->motorStatusMoving_, 1);
        setIntegerParam(this->pC_->motorStatusDirection_,
                        (lastStatus_ & SB_INMOTIONCW) ? 0 : 1);
        setIntegerParam(this->pC_->motorStatusDone_, 0);
        *moving = true;
        APT_INFO("poll: MOVING %s\n",
                 (lastStatus_ & SB_INMOTIONCW) ? "CW" : "CCW");
    } else {
        setIntegerParam(this->pC_->motorStatusMoving_, 0);
        setIntegerParam(this->pC_->motorStatusDone_, 1);
        *moving = false;
    }

    setIntegerParam(this->pC_->motorStatusLowLimit_,
                    (lastStatus_ & SB_CWHARDLIMIT) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHighLimit_,
                    (lastStatus_ & SB_CCWHARDLIMIT) ? 1 : 0);
    /* SB_HOMED (0x400): set and latched by the controller after a successful
     * home move.  This is the correct bit for motorStatusHome_ (→ ATHM) and
     * motorStatusHomed_ (→ RA_HOMED).  SB_HOMING (0x200) is active only
     * while the home move is running and clears on completion. */
    setIntegerParam(this->pC_->motorStatusHome_,
                    (lastStatus_ & SB_HOMED) ? 1 : 0);
    setIntegerParam(this->pC_->motorStatusHomed_,
                    (lastStatus_ & SB_HOMED) ? 1 : 0);

    if (lastStatus_ & SB_ERROR)
        APT_ERR("poll: controller reports ERROR bit set (0x%08X)\n", lastStatus_);

    callParamCallbacks();

    return asynSuccess;
}

asynStatus AptAxis::stop(double acceleration)
{
    APT_INFO("stop: sending immediate stop\n");
    if (pC_->aptSerial_) {
        pC_->aptLock_.lock();
        aptSerialFlush(pC_->aptSerial_);
        pC_->aptLock_.unlock();
    }
    this->stopImmediate();
    return asynSuccess;
}

/* Default implementations for APT operations */

void AptAxis::enableChannel()
{
    if (!pC_->aptSerial_) return;
    APT_INFO("enableChannel: ch=%d\n", channel_);
    pC_->aptLock_.lock();
    aptSendShortMessage(pC_->aptSerial_, MGMSG_MOD_SET_CHANENABLESTATE,
                        channel_, APT_CHAN_ENABLE,
                        pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

void AptAxis::disableChannel()
{
    if (!pC_->aptSerial_) return;
    APT_INFO("disableChannel: ch=%d\n", channel_);
    pC_->aptLock_.lock();
    aptSendShortMessage(pC_->aptSerial_, MGMSG_MOD_SET_CHANENABLESTATE,
                        channel_, APT_CHAN_DISABLE,
                        pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

int AptAxis::getPosition()
{
    /* Request position counter */
    uint8_t data[256];
    uint16_t dataLen = 0;
    int ret = requestStatus(MGMSG_MOT_REQ_POSCOUNTER,
                            MGMSG_MOT_GET_POSCOUNTER,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptPosCounter)) {
        AptPosCounter* pos = (AptPosCounter*)data;
        APT_TRACE("getPosition: pos=%d (dataLen=%u)\n", pos->position, dataLen);
        return pos->position;
    }
    APT_ERR("getPosition: FAILED ret=%d dataLen=%u\n", ret, dataLen);
    return lastPosition_;  /* return last known, not 0 */
}

uint32_t AptAxis::getStatus()
{
    /* Request status bits */
    uint8_t data[256];
    uint16_t dataLen = 0;
    int ret = requestStatus(MGMSG_MOT_REQ_STATUSBITS,
                            MGMSG_MOT_GET_STATUSBITS,
                            data, sizeof(data), &dataLen);
    if (ret == 0 && dataLen >= sizeof(AptStatusBits)) {
        AptStatusBits* sb = (AptStatusBits*)data;
        APT_TRACE("getStatus: bits=0x%08X\n", sb->statusBits);
        return sb->statusBits;
    }
    APT_ERR("getStatus: FAILED ret=%d dataLen=%u\n", ret, dataLen);
    return lastStatus_;  /* return last known, not 0 */
}

void AptAxis::setVelParams(int32_t acceleration, int32_t maxVelocity)
{
    if (!pC_->aptSerial_) return;

    APT_INFO("setVelParams: accel=%d maxVel=%d\n", acceleration, maxVelocity);

    AptVelParams params;
    params.chanIdent = channel_;
    params.minVelocity = 0;
    params.acceleration = acceleration;
    params.maxVelocity = maxVelocity;

    pC_->aptLock_.lock();
    aptSendLongMessage(pC_->aptSerial_, MGMSG_MOT_SET_VELPARAMS,
                       &params, sizeof(params),
                       pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

void AptAxis::moveRelative(int32_t distance)
{
    if (!pC_->aptSerial_) return;

    APT_INFO("moveRelative: distance=%d counts\n", distance);

    AptMoveParams params;
    params.chanIdent = channel_;
    params.distance = distance;

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_MOT_MOVE_RELATIVE,
                                 &params, sizeof(params),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("moveRelative: aptSendLongMessage FAILED (ret=%d)\n", ret);
    else
        APT_INFO("moveRelative: command sent OK\n");
}

void AptAxis::moveToPosition(int32_t position)
{
    if (!pC_->aptSerial_) return;

    APT_INFO("moveToPosition: target=%d counts\n", position);

    AptMoveParams params;
    params.chanIdent = channel_;
    params.distance = position;

    pC_->aptLock_.lock();
    int ret = aptSendLongMessage(pC_->aptSerial_, MGMSG_MOT_MOVE_ABSOLUTE,
                                 &params, sizeof(params),
                                 pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();

    if (ret != 0)
        APT_ERR("moveToPosition: aptSendLongMessage FAILED (ret=%d)\n", ret);
    else
        APT_INFO("moveToPosition: command sent OK\n");
}

void AptAxis::doHome()
{
    if (!pC_->aptSerial_) return;
    APT_INFO("doHome: ch=%d\n", channel_);
    pC_->aptLock_.lock();
    aptSendShortMessage(pC_->aptSerial_, MGMSG_MOT_MOVE_HOME,
                        channel_, 0x00,
                        pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
}

int AptAxis::stopImmediate()
{
    if (!pC_->aptSerial_) return -1;
    APT_INFO("stopImmediate: ch=%d\n", channel_);
    pC_->aptLock_.lock();
    int ret = aptSendShortMessage(pC_->aptSerial_, MGMSG_MOT_MOVE_STOP,
                                  channel_, APT_STOP_IMMEDIATE,
                                  pC_->dest_, APT_HOST);
    pC_->aptLock_.unlock();
    return ret;
}

/* ========================================================================= */
/*  IOC Shell registration                                                    */
/* ========================================================================= */

extern "C" void AptControllerConfig(const char* asyn_port,
                                     const char* devicePath,
                                     const char* type,
                                     int numAxes,
                                     double movingPollPeriod,
                                     double idlePollPeriod)
{
    std::string motor_type(type);
    int motorType;

    if (motor_type == "dc" || motor_type == "DC") {
        motorType = APT_DC_MOTOR;
    } else if (motor_type == "stepper" || motor_type == "Stepper" ||
               motor_type == "step" || motor_type == "Step") {
        motorType = APT_STEP_MOTOR;
    } else {
        printf("AptControllerConfig: Unknown motor type: %s\n", type);
        printf("  Valid types: dc, DC, stepper, step\n");
        return;
    }

    new AptController(asyn_port, devicePath, motorType, numAxes,
                      movingPollPeriod, idlePollPeriod);
}

/* -------------------------------------------------------------------------- */
/*  AptSetVelParams — set velocity parameters directly in APT internal units  */
/* -------------------------------------------------------------------------- */

extern "C" void AptSetVelParams(const char* asyn_port,
                                 int axisNum,
                                 int minVel,
                                 int accel,
                                 int maxVel)
{
    AptController* pC = (AptController*)findAsynPortDriver(asyn_port);
    if (!pC) {
        printf("AptSetVelParams: port '%s' not found\n", asyn_port);
        return;
    }

    AptAxis* ax = pC->getAxis(axisNum);
    if (!ax) {
        printf("AptSetVelParams: axis %d not found on port '%s'\n",
               axisNum, asyn_port);
        return;
    }

    printf("AptSetVelParams: setting minV=%d accel=%d maxV=%d "
           "(APT internal units) on port '%s' axis %d\n",
           minVel, accel, maxVel, asyn_port, axisNum);

    ax->setVelParams(accel, maxVel);
}

static const iocshArg AptSetVelParamsArg0 = {"Motor Port Name", iocshArgString};
static const iocshArg AptSetVelParamsArg1 = {"Axis Number", iocshArgInt};
static const iocshArg AptSetVelParamsArg2 = {"minVelocity", iocshArgInt};
static const iocshArg AptSetVelParamsArg3 = {"acceleration", iocshArgInt};
static const iocshArg AptSetVelParamsArg4 = {"maxVelocity", iocshArgInt};

static const iocshArg* const AptSetVelParamsArgs[] = {
    &AptSetVelParamsArg0, &AptSetVelParamsArg1,
    &AptSetVelParamsArg2, &AptSetVelParamsArg3,
    &AptSetVelParamsArg4
};

static const iocshFuncDef AptSetVelParamsDef = {
    "AptSetVelParams", 5, AptSetVelParamsArgs
};

static void AptSetVelParamsCallFunc(const iocshArgBuf* args)
{
    AptSetVelParams(args[0].sval, args[1].ival, args[2].ival,
                    args[3].ival, args[4].ival);
}

static const iocshArg AptControllerArg0 = {"Motor Port Name", iocshArgString};
static const iocshArg AptControllerArg1 = {"Device Path", iocshArgString};
static const iocshArg AptControllerArg2 = {"Motor Type", iocshArgString};
static const iocshArg AptControllerArg3 = {"Num Axes (0=auto)", iocshArgInt};
static const iocshArg AptControllerArg4 = {"movingPollPeriod", iocshArgDouble};
static const iocshArg AptControllerArg5 = {"idlePollPeriod", iocshArgDouble};

static const iocshArg* const AptControllerArgs[] = {
    &AptControllerArg0, &AptControllerArg1, &AptControllerArg2,
    &AptControllerArg3, &AptControllerArg4, &AptControllerArg5
};

static const iocshFuncDef AptControllerDef = {
    "AptControllerConfig", 6, AptControllerArgs
};

static void AptControllerCallFunc(const iocshArgBuf* args)
{
    AptControllerConfig(args[0].sval, args[1].sval, args[2].sval,
                        args[3].ival, args[4].dval, args[5].dval);
}

static void AptRegister(void)
{
    iocshRegister(&AptControllerDef, AptControllerCallFunc);
    iocshRegister(&AptSetVelParamsDef, AptSetVelParamsCallFunc);
}

extern "C"
{
    epicsExportRegistrar(AptRegister);
}
