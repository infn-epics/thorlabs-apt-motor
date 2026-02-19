/*
 * aptSerial.h
 *
 * Transport layer for Thorlabs APT protocol on Linux.
 *
 * Supports two transport modes:
 *
 *   1. Local USB serial  — FTDI USB-to-serial via POSIX termios.
 *      Device path:  "/dev/ttyUSB0"  (any path starting with '/')
 *
 *   2. Remote via ser2net — TCP socket connection to a ser2net daemon
 *      that bridges TCP to the FTDI serial port on a remote host.
 *      Address format:  "hostname:port"  or  "192.168.1.100:4001"
 *
 * Communication parameters (configured locally for serial, or in the
 * ser2net configuration file for the remote case):
 *   - 115200 baud, 8 data bits, 1 stop bit, no parity
 *   - RTS/CTS hardware flow control
 */

#ifndef INC_APT_SERIAL_H
#define INC_APT_SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include "aptProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque handle for an APT transport connection (serial or TCP).
 */
typedef struct AptSerial AptSerial;

/**
 * Open a connection to a Thorlabs APT controller.
 *
 * The function auto-detects the transport mode from the address:
 *   - Starts with '/'       → local serial device (e.g. "/dev/ttyUSB0")
 *   - Contains 'host:port'  → TCP connection to ser2net (e.g. "192.168.1.100:4001")
 *
 * @param devicePath  Serial device path or host:port for ser2net
 * @return Handle on success, NULL on failure
 */
AptSerial* aptSerialOpen(const char* devicePath);

/**
 * Close the serial connection and free resources.
 */
void aptSerialClose(AptSerial* apt);

/**
 * Send a short (header-only, 6-byte) APT message.
 *
 * @return 0 on success, -1 on error
 */
int aptSendShortMessage(AptSerial* apt, uint16_t msgId,
                        uint8_t param1, uint8_t param2,
                        uint8_t dest, uint8_t source);

/**
 * Send a long APT message (6-byte header + data payload).
 *
 * @param data       Pointer to data payload
 * @param dataLen    Length of data payload in bytes
 * @return 0 on success, -1 on error
 */
int aptSendLongMessage(AptSerial* apt, uint16_t msgId,
                       const void* data, uint16_t dataLen,
                       uint8_t dest, uint8_t source);

/**
 * Receive the next APT message.
 *
 * Reads the 6-byte header first. If it's a long message, also reads
 * the data payload into the provided buffer.
 *
 * @param header     Output: 6-byte header
 * @param data       Output buffer for data payload (if any)
 * @param dataMaxLen Maximum data buffer size
 * @param dataLen    Output: actual data length read (0 for short messages)
 * @param timeoutMs  Timeout in milliseconds (0 = blocking)
 * @return 0 on success, -1 on error, -2 on timeout
 */
int aptReceiveMessage(AptSerial* apt, uint8_t* header,
                      void* data, uint16_t dataMaxLen, uint16_t* dataLen,
                      int timeoutMs);

/**
 * Flush any pending input/output data.
 */
void aptSerialFlush(AptSerial* apt);

/**
 * Get the file descriptor (for select/poll in advanced use).
 */
int aptSerialGetFd(AptSerial* apt);

#ifdef __cplusplus
}
#endif

#endif /* INC_APT_SERIAL_H */
