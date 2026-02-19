/*
 * aptProtocol.h
 *
 * Thorlabs APT (Advanced Positioning Technology) protocol definitions.
 * Implements the binary protocol for communicating with Thorlabs motion
 * controllers over USB-to-serial (FTDI) links on Linux.
 *
 * Based on: "Thorlabs Motion Controllers Host-Controller Communications Protocol"
 *           Document dated 9 December 2025
 *
 * Author: Auto-generated Linux port from Kinesis Windows driver
 * Date:   February 2026
 */

#ifndef INC_APT_PROTOCOL_H
#define INC_APT_PROTOCOL_H

#include <stdint.h>
#include <string.h>

/*
 * APT Message IDs
 */

/* Hardware / System messages */
#define MGMSG_HW_DISCONNECT          0x0002
#define MGMSG_HW_REQ_INFO            0x0005
#define MGMSG_HW_GET_INFO            0x0006
#define MGMSG_HW_START_UPDATEMSGS    0x0011
#define MGMSG_HW_STOP_UPDATEMSGS     0x0012
#define MGMSG_RACK_REQ_BAYUSED       0x0060
#define MGMSG_RACK_GET_BAYUSED       0x0061
#define MGMSG_HW_RESPONSE            0x0080
#define MGMSG_HW_RICHRESPONSE        0x0081

/* Module messages */
#define MGMSG_MOD_IDENTIFY            0x0223
#define MGMSG_MOD_SET_CHANENABLESTATE 0x0210
#define MGMSG_MOD_REQ_CHANENABLESTATE 0x0211
#define MGMSG_MOD_GET_CHANENABLESTATE 0x0212

/* Motor position counters */
#define MGMSG_MOT_SET_POSCOUNTER     0x0410
#define MGMSG_MOT_REQ_POSCOUNTER     0x0411
#define MGMSG_MOT_GET_POSCOUNTER     0x0412

/* Motor encoder counters */
#define MGMSG_MOT_SET_ENCCOUNTER     0x0409
#define MGMSG_MOT_REQ_ENCCOUNTER     0x040A
#define MGMSG_MOT_GET_ENCCOUNTER     0x040B

/* Velocity parameters */
#define MGMSG_MOT_SET_VELPARAMS      0x0413
#define MGMSG_MOT_REQ_VELPARAMS      0x0414
#define MGMSG_MOT_GET_VELPARAMS      0x0415

/* Jog parameters */
#define MGMSG_MOT_SET_JOGPARAMS      0x0416
#define MGMSG_MOT_REQ_JOGPARAMS      0x0417
#define MGMSG_MOT_GET_JOGPARAMS      0x0418

/* Limit switch parameters */
#define MGMSG_MOT_SET_LIMSWITCHPARAMS 0x0423
#define MGMSG_MOT_REQ_LIMSWITCHPARAMS 0x0424
#define MGMSG_MOT_GET_LIMSWITCHPARAMS 0x0425

/* Power parameters */
#define MGMSG_MOT_SET_POWERPARAMS    0x0426
#define MGMSG_MOT_REQ_POWERPARAMS    0x0427
#define MGMSG_MOT_GET_POWERPARAMS    0x0428

/* Status bits */
#define MGMSG_MOT_REQ_STATUSBITS     0x0429
#define MGMSG_MOT_GET_STATUSBITS     0x042A

/* General move parameters */
#define MGMSG_MOT_SET_GENMOVEPARAMS  0x043A
#define MGMSG_MOT_REQ_GENMOVEPARAMS  0x043B
#define MGMSG_MOT_GET_GENMOVEPARAMS  0x043C

/* Home parameters and commands */
#define MGMSG_MOT_SET_HOMEPARAMS     0x0440
#define MGMSG_MOT_REQ_HOMEPARAMS     0x0441
#define MGMSG_MOT_GET_HOMEPARAMS     0x0442
#define MGMSG_MOT_MOVE_HOME          0x0443
#define MGMSG_MOT_MOVE_HOMED         0x0444

/* Relative move */
#define MGMSG_MOT_SET_MOVERELPARAMS  0x0445
#define MGMSG_MOT_REQ_MOVERELPARAMS  0x0446
#define MGMSG_MOT_GET_MOVERELPARAMS  0x0447
#define MGMSG_MOT_MOVE_RELATIVE      0x0448

/* Absolute move */
#define MGMSG_MOT_SET_MOVEABSPARAMS  0x0450
#define MGMSG_MOT_REQ_MOVEABSPARAMS  0x0451
#define MGMSG_MOT_GET_MOVEABSPARAMS  0x0452
#define MGMSG_MOT_MOVE_ABSOLUTE      0x0453

/* Move velocity */
#define MGMSG_MOT_MOVE_VELOCITY      0x0457

/* Move completed / stopped */
#define MGMSG_MOT_MOVE_COMPLETED     0x0464
#define MGMSG_MOT_MOVE_STOP          0x0465
#define MGMSG_MOT_MOVE_STOPPED       0x0466

/* Jog */
#define MGMSG_MOT_MOVE_JOG           0x046A

/* Suspend/resume end of move messages */
#define MGMSG_MOT_SUSPEND_ENDOFMOVEMSGS 0x046B
#define MGMSG_MOT_RESUME_ENDOFMOVEMSGS  0x046C

/* Status updates */
#define MGMSG_MOT_REQ_STATUSUPDATE   0x0480
#define MGMSG_MOT_GET_STATUSUPDATE   0x0481
#define MGMSG_MOT_REQ_USTATUSUPDATE  0x0490
#define MGMSG_MOT_GET_USTATUSUPDATE  0x0491
#define MGMSG_MOT_ACK_USTATUSUPDATE  0x0492

/* DC PID parameters */
#define MGMSG_MOT_SET_DCPIDPARAMS    0x04A0
#define MGMSG_MOT_REQ_DCPIDPARAMS    0x04A1
#define MGMSG_MOT_GET_DCPIDPARAMS    0x04A2

/* EEPROM */
#define MGMSG_MOT_SET_EEPROMPARAMS   0x04B9

/*
 * Source/Destination IDs
 */
#define APT_HOST           0x01   /* Host PC */
#define APT_RACK           0x11   /* Rack / motherboard */
#define APT_BAY0           0x21   /* Bay 0 (0x21=Bay0, 0x22=Bay1, ..., 0x2A=Bay9) */
#define APT_BAY(n)         (APT_BAY0 + (n))   /* Bay address for bay n (0-based) */
#define APT_GENERIC_USB    0x50   /* Generic USB unit (single-channel) */

/*
 * Channel enable states
 */
#define APT_CHAN_ENABLE     0x01
#define APT_CHAN_DISABLE    0x02

/*
 * Channel identifiers
 */
#define APT_CHANNEL_1      0x01
#define APT_CHANNEL_2      0x02
#define APT_CHANNEL_3      0x03
#define APT_CHANNEL_4      0x04

/*
 * Maximum number of axes per controller (for array sizing).
 * Bay-type systems (BSC103, BBD10x) have up to 3-6 bays;
 * 10 is a safe upper bound.
 */
#define APT_MAX_AXES       10

/*
 * Stop modes
 */
#define APT_STOP_IMMEDIATE 0x01
#define APT_STOP_PROFILED  0x02

/*
 * Status bits definitions
 */
#define SB_CWHARDLIMIT      0x00000001
#define SB_CCWHARDLIMIT     0x00000002
#define SB_CWSOFTLIMIT      0x00000004
#define SB_CCWSOFTLIMIT     0x00000008
#define SB_INMOTIONCW       0x00000010
#define SB_INMOTIONCCW      0x00000020
#define SB_JOGGINGCW        0x00000040
#define SB_JOGGINGCCW       0x00000080
#define SB_CONNECTED        0x00000100
#define SB_HOMING           0x00000200
#define SB_HOMED            0x00000400
#define SB_INITIALIZING     0x00000800
#define SB_TRACKING         0x00001000
#define SB_SETTLED          0x00002000
#define SB_POSITIONERROR    0x00004000
#define SB_INSTRERROR       0x00008000
#define SB_INTERLOCK        0x00010000
#define SB_OVERTEMP         0x00020000
#define SB_BUSVOLTFAULT     0x00040000
#define SB_COMMUTATIONERR   0x00080000
#define SB_OVERLOAD         0x01000000
#define SB_ENCODERFAULT     0x02000000
#define SB_OVERCURRENT      0x04000000
#define SB_BUSCURRENTFAULT  0x08000000
#define SB_POWEROK          0x10000000
#define SB_ACTIVE           0x20000000
#define SB_ERROR            0x40000000
#define SB_ENABLED          0x80000000

/*
 * APT message header (always 6 bytes)
 *
 * For header-only messages:
 *   bytes 0-1: message ID (little-endian)
 *   byte  2:   param1
 *   byte  3:   param2
 *   byte  4:   dest
 *   byte  5:   source
 *
 * For messages with data:
 *   bytes 0-1: message ID (little-endian)
 *   bytes 2-3: data packet length (little-endian)
 *   byte  4:   dest | 0x80
 *   byte  5:   source
 */

#pragma pack(push, 1)

struct AptHeader {
    uint16_t msgId;
    union {
        struct {
            uint8_t param1;
            uint8_t param2;
            uint8_t dest;
            uint8_t source;
        } shortMsg;
        struct {
            uint16_t dataLength;
            uint8_t  destOrFlag;   /* dest | 0x80 */
            uint8_t  source;
        } longMsg;
    };
};

/* HW_GET_INFO response data (84 bytes) */
struct AptHwInfo {
    uint32_t serialNumber;
    char     modelNumber[8];
    uint16_t type;
    uint8_t  firmwareMinor;
    uint8_t  firmwareInterim;
    uint8_t  firmwareMajor;
    uint8_t  firmwareUnused;
    uint8_t  internalUse[60];
    uint16_t hwVersion;
    uint16_t modState;
    uint16_t numChannels;
};

/* Velocity parameters data (14 bytes) */
struct AptVelParams {
    uint16_t chanIdent;
    int32_t  minVelocity;
    int32_t  acceleration;
    int32_t  maxVelocity;
};

/* Position counter data (6 bytes) */
struct AptPosCounter {
    uint16_t chanIdent;
    int32_t  position;
};

/* Relative/Absolute move data (6 bytes) */
struct AptMoveParams {
    uint16_t chanIdent;
    int32_t  distance;
};

/* Status update (14 bytes) - used with USTATUSUPDATE */
struct AptUStatusUpdate {
    uint16_t chanIdent;
    int32_t  position;
    uint16_t velocity;
    uint16_t motorCurrent;
    uint32_t statusBits;
};

/* Full status update (14 bytes) - used with STATUSUPDATE for stepper */
struct AptStatusUpdate {
    uint16_t chanIdent;
    int32_t  position;
    int32_t  encoderCount;
    uint32_t statusBits;
};

/* Status bits response (6 bytes) */
struct AptStatusBits {
    uint16_t chanIdent;
    uint32_t statusBits;
};

#pragma pack(pop)

/*
 * Helper functions for building APT messages
 */

/**
 * Build a 6-byte header-only APT message.
 */
static inline void aptBuildShortMessage(uint8_t* buf, uint16_t msgId,
                                         uint8_t param1, uint8_t param2,
                                         uint8_t dest, uint8_t source)
{
    buf[0] = (uint8_t)(msgId & 0xFF);
    buf[1] = (uint8_t)((msgId >> 8) & 0xFF);
    buf[2] = param1;
    buf[3] = param2;
    buf[4] = dest;
    buf[5] = source;
}

/**
 * Build a 6-byte header for a long message (with following data payload).
 */
static inline void aptBuildLongHeader(uint8_t* buf, uint16_t msgId,
                                       uint16_t dataLength,
                                       uint8_t dest, uint8_t source)
{
    buf[0] = (uint8_t)(msgId & 0xFF);
    buf[1] = (uint8_t)((msgId >> 8) & 0xFF);
    buf[2] = (uint8_t)(dataLength & 0xFF);
    buf[3] = (uint8_t)((dataLength >> 8) & 0xFF);
    buf[4] = dest | 0x80;
    buf[5] = source;
}

/**
 * Check if an APT message header indicates a long message (with data payload).
 */
static inline bool aptIsLongMessage(const uint8_t* header)
{
    return (header[4] & 0x80) != 0;
}

/**
 * Extract message ID from header buffer.
 */
static inline uint16_t aptGetMsgId(const uint8_t* header)
{
    return (uint16_t)(header[0] | (header[1] << 8));
}

/**
 * Extract data length from long message header.
 */
static inline uint16_t aptGetDataLength(const uint8_t* header)
{
    return (uint16_t)(header[2] | (header[3] << 8));
}

#endif /* INC_APT_PROTOCOL_H */
