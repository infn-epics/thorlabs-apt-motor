/*
 * drvApt.h
 *
 * EPICS asynMotor driver for Thorlabs APT controllers on Linux.
 * Replaces the Kinesis Windows library with direct APT protocol
 * communication over USB-to-serial (FTDI) or TCP (ser2net).
 *
 * Supports:
 *   - DC servo motors (TDC001, KDC101, etc.)
 *   - Stepper motors (TST001, KST101, BSC10x, BSC20x, etc.)
 *   - Multi-channel bay controllers (BSC102, BSC103, BBD10x, BBD20x, BBD30x)
 *   - Local USB serial (/dev/ttyUSBx)
 *   - Remote via ser2net (host:port)
 *
 * The driver uses the same EPICS motor interface as the Windows
 * Kinesis driver, so existing IOC databases and startup scripts
 * need minimal changes (just the device path instead of serial number).
 *
 * Debug levels (set via IOC shell: var drvAptDebug <level>):
 *   0 = off (default)
 *   1 = errors and connection events
 *   2 = move commands, poll summary (position, status bits)
 *   3 = APT message-level trace (request/response IDs)
 *   4 = raw hex dump of all sent/received bytes
 */

#ifndef INC_DRVAPT_H
#define INC_DRVAPT_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "aptProtocol.h"
#include "aptSerial.h"

#include <epicsThread.h>
#include <epicsMutex.h>

/* ---------- debug infrastructure ---------- */
extern "C" { extern int drvAptDebug; }

#define APT_DBG(level, fmt, ...) \
    do { if (drvAptDebug >= (level)) \
        printf("drvApt[%d]: " fmt, (level), ##__VA_ARGS__); \
    } while(0)

/* convenience per-level macros */
#define APT_ERR(fmt, ...)   APT_DBG(1, fmt, ##__VA_ARGS__)
#define APT_INFO(fmt, ...)  APT_DBG(2, fmt, ##__VA_ARGS__)
#define APT_TRACE(fmt, ...) APT_DBG(3, fmt, ##__VA_ARGS__)
#define APT_RAW(fmt, ...)   APT_DBG(4, fmt, ##__VA_ARGS__)

enum AptMotorType
{
    APT_DC_MOTOR,
    APT_STEP_MOTOR,
    APT_PIEZO_MOTOR,
    APT_PZMOT_MOTOR,   /* KIM101, TIM101 — Piezo Inertial Motor (PZMOT_*) */
};

class epicsShareClass AptController;

/**
 * Base axis class for APT controllers.
 * Handles common motor operations via APT protocol.
 */
class epicsShareClass AptAxis : public asynMotorAxis
{
public:
    AptAxis(AptController* controller, int axisNo, uint8_t channel);

    /* asynMotorAxis interface */
    asynStatus move(double position, int relative, double min_velocity,
                    double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity,
                    double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool* moving);

    /* APT operations implemented by subclasses */
    virtual void enableChannel();
    virtual void disableChannel();
    virtual int  getPosition();
    virtual uint32_t getStatus();
    virtual void setVelParams(int32_t acceleration, int32_t maxVelocity);
    virtual void moveRelative(int32_t distance);
    virtual void moveToPosition(int32_t position);
    virtual void doHome();
    virtual int  stopImmediate();

protected:
    AptController* pC_;
    uint8_t channel_;

    /* Cached values from last successful poll */
    int32_t  lastPosition_;
    uint32_t lastStatus_;
    bool     pollOk_;        /* true after at least one successful poll */

    /** Request and wait for a specific response message */
    int requestStatus(uint16_t reqMsgId, uint16_t getMsgId,
                      void* data, uint16_t dataMaxLen, uint16_t* dataLen);
};

/**
 * DC servo motor axis (TDC001, KDC101, etc.).
 * Uses MGMSG_MOT_GET_USTATUSUPDATE for status.
 */
class epicsShareClass AptDCMotorAxis : public AptAxis
{
public:
    AptDCMotorAxis(AptController* controller, int axisNo, uint8_t channel);
    ~AptDCMotorAxis();

    /* Override poll to use single USTATUSUPDATE for pos+status */
    asynStatus poll(bool* moving);

    int  getPosition();
    uint32_t getStatus();
};

/**
 * Stepper motor axis (TST001, KST101, BSC10x, BSC20x, KIM101, etc.).
 * Auto-detects which status message the controller supports:
 *   1. USTATUSUPDATE  (pos+vel+cur+status)   — DC-style controllers
 *   2. STATUSUPDATE   (pos+enc+status)        — legacy stepper
 *   3. POSCOUNTER + STATUSBITS (two requests) — universal (KIM101, etc.)
 */
class epicsShareClass AptStepMotorAxis : public AptAxis
{
public:
    AptStepMotorAxis(AptController* controller, int axisNo, uint8_t channel);
    ~AptStepMotorAxis();

    asynStatus poll(bool* moving);

    int  getPosition();
    uint32_t getStatus();

private:
    /**
     * Poll method cache — avoids repeated 2-second timeouts on
     * unsupported message types.
     *   0 = auto-detect (try all methods in order)
     *   1 = USTATUSUPDATE  (0x0490/0x0491)
     *   2 = STATUSUPDATE   (0x0480/0x0481)
     *   3 = POSCOUNTER + STATUSBITS (0x0411 + 0x0429)
     */
    int pollMethod_;
};

/**
 * Piezo axis (KPZ101, TPZ001, etc.).
 * Uses MGMSG_PZ_* messages for open-loop voltage or closed-loop position.
 * Position range: 0–32767 (maps to 0–100% of max travel).
 */
class epicsShareClass AptPiezoAxis : public AptAxis
{
public:
    AptPiezoAxis(AptController* controller, int axisNo, uint8_t channel);
    ~AptPiezoAxis();

    /* asynMotorAxis interface overrides */
    asynStatus move(double position, int relative, double min_velocity,
                    double max_velocity, double acceleration);
    asynStatus poll(bool* moving);
    asynStatus stop(double acceleration);
    asynStatus home(double min_velocity, double max_velocity,
                    double acceleration, int forwards);

    /* Piezo-specific operations */
    void setPositionControlMode(uint8_t mode);
    void setOutputPosition(uint16_t pos);
    uint16_t getOutputPosition();
    void setOutputVoltage(int16_t volts);

private:
    uint16_t lastPzPosition_;  /* 0-32767 */
    uint32_t lastPzStatus_;
    uint8_t  controlMode_;     /* current control mode */
};

/**
 * Piezo Inertial Motor axis (KIM101, TIM101, KIM001).
 *
 * Uses MGMSG_PZMOT_* messages (0x08xx range) — a completely separate
 * protocol family from both MOT_* (stepper/DC) and PZ_* (direct piezo).
 *
 * Key differences from other motor types:
 *   - Channel ident uses bitmask encoding (1,2,4,8) in data messages
 *   - Status update returns ALL 4 channels in one 56-byte response
 *   - Requires ACK_STATUSUPDATE keepalive every 10 status updates
 *   - Position is in actuator steps (not encoder counts or %)
 *   - Stop uses standard MOT_MOVE_STOP (0x0465)
 *   - Home uses standard MOT_MOVE_HOME (0x0443)
 */
class epicsShareClass AptPzMotAxis : public AptAxis
{
public:
    AptPzMotAxis(AptController* controller, int axisNo, uint8_t channel);
    ~AptPzMotAxis();

    /* asynMotorAxis interface overrides */
    asynStatus move(double position, int relative, double min_velocity,
                    double max_velocity, double acceleration);
    asynStatus poll(bool* moving);
    asynStatus stop(double acceleration);
    asynStatus home(double min_velocity, double max_velocity,
                    double acceleration, int forwards);

    /* PZMOT-specific operations */
    void pzMotEnableChannel();
    void pzMotMoveAbsolute(int32_t position);
    void pzMotMoveJog(int direction);
    int  pzMotRequestStatus(AptPzMotStatusUpdate* status);

private:
    uint16_t chanBitmask_;    /* channel bitmask (1, 2, 4, 8) */
    uint16_t chanEnableMode_; /* PZMOT channel enable value (1-4) */
    int      ackCounter_;     /* counter for ACK keepalive */
};

/**
 * EPICS motor controller for Thorlabs APT devices.
 *
 * Supports single-channel USB controllers (TDC001, KDC101, KST101, KPZ101, ...)
 * and multi-channel bay/rack controllers (BSC102, BSC103, BBD10x, ...).
 *
 * For single-channel (numAxes=1): dest = APT_GENERIC_USB (0x50)
 * For multi-channel (numAxes>1): dest = APT_RACK (0x11), channels addressed
 *   via ChanIdent in each APT data message.
 */
class epicsShareClass AptController : public asynMotorController
{
public:
    /**
     * @param portName    EPICS asyn port name
     * @param devicePath  Connection address:
     *                    - Local serial device (e.g. /dev/ttyUSB0)
     *                    - Remote ser2net endpoint (e.g. 192.168.1.100:4001)
     * @param type        Motor type (APT_DC_MOTOR or APT_STEP_MOTOR)
     * @param numAxes     Number of motor axes:
     *                    - 0: auto-detect from HW_GET_INFO numChannels
     *                    - 1: single-channel USB device (backward compatible)
     *                    - N>1: multi-channel bay controller with N channels
     * @param movingPollPeriod  Polling period when moving (sec)
     * @param idlePollPeriod    Polling period when idle (sec)
     */
    AptController(const char* portName, const char* devicePath,
                  int type, int numAxes,
                  double movingPollPeriod, double idlePollPeriod);
    ~AptController();

    AptAxis* getAxis(asynUser* pasynuser);
    AptAxis* getAxis(int axis);

    /** Get the APT serial connection handle */
    AptSerial* getAptSerial() { return aptSerial_; }

    /** Get the destination byte for messages */
    uint8_t getDest() { return dest_; }

    /** Mutex for serializing APT communication */
    epicsMutex aptLock_;

    friend class AptAxis;
    friend class AptDCMotorAxis;
    friend class AptStepMotorAxis;
    friend class AptPiezoAxis;
    friend class AptPzMotAxis;

private:
    AptSerial* aptSerial_;
    uint8_t    dest_;        /* APT_GENERIC_USB (single) or APT_RACK (multi) */
    int        numAxes_;     /* Actual number of axes created */
    int        motorType_;   /* APT_DC_MOTOR or APT_STEP_MOTOR */
    char       devicePath_[256];
};

#endif /* INC_DRVAPT_H */
