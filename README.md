# Thorlabs Linux APT IOC

Standalone EPICS IOC for Thorlabs motion controllers on Linux.

Uses the APT (Advanced Positioning Technology) binary serial protocol
to communicate with controllers over local USB serial (FTDI) or remotely
via ser2net (TCP).

Supports **single-channel** USB units, **multi-channel bay/rack** controllers
(BSC102/103, BBD10x/20x/30x), **direct-drive piezo** controllers (KPZ101,
TPZ001), and **multi-channel piezo inertial** motor controllers (KIM101,
TIM101). The number of axes is either specified explicitly or auto-detected
from the controller's `HW_GET_INFO` response at startup.

## Supported Controllers

### Single-Channel DC Servo Motor Controllers
- **TDC001** — T-Cube DC Servo Motor Controller
- **KDC001** — K-Cube DC Servo Motor Controller  
- **KDC101** — K-Cube Brushed DC Servo Motor Controller
- **ODC001** — OptoDriver (mini DC servo driver)

### Multi-Channel DC Servo Bay Controllers
- **BBD102** — Benchtop Brushless DC Controller (2 bays)
- **BBD103 / BBD203 / BBD303** — Benchtop Brushless DC Controller (3 bays)
- **TBD001 / KBD101** — T-Cube / K-Cube Brushless DC Controller

### Single-Channel Stepper Motor Controllers
- **TST001** — T-Cube Stepper Motor Controller
- **KST101** — K-Cube Stepper Motor Controller
- **BSC101 / BSC201** — Benchtop Stepper Motor Controller (1 channel)
- **LTS150 / LTS300** — Long Travel Stages

### Multi-Channel Stepper Motor Bay Controllers
- **BSC102 / BSC202** — Benchtop Stepper Motor Controller (2 bays)
- **BSC103 / BSC203** — Benchtop Stepper Motor Controller (3 bays)

### Single-Channel Direct-Drive Piezo Controllers
- **KPZ101** — K-Cube Piezo Controller (open/closed loop, ±75 V)
- **TPZ001** — T-Cube Piezo Controller
- **MPZ601** — Multi-Channel Piezo Controller

These use the APT `PZ_*` message family (0x06xx). Position is a 16-bit
percent-of-travel value (0–32767). Use `motorType="piezo"`.

### Multi-Channel Piezo Inertial Motor Controllers
- **KIM101** — K-Cube Piezo Inertial Motor Controller (4 channels)
- **TIM101** — T-Cube Piezo Inertial Motor Controller (4 channels)

These use the APT `PZMOT_*` message family (0x08xx), which is completely
distinct from both the motor (MOT_*) and direct-drive piezo (PZ_*) families.
All 4 channels are addressed through a single USB connection. Use
`motorType="kim"` (aliases: `"pzmot"`, `"tim"`). Position is in encoder
counts (step increments). **Homing is not supported** by KIM101/TIM101
hardware (no home switch); the EPICS `home` command zeros the position counter
at the current physical position instead.

## Prerequisites

- EPICS Base 7.x
- Motor module (synApps motor)
- Asyn module

## Building

1. Edit `configure/RELEASE` to point to your EPICS base, motor, and asyn
   installation paths.

2. Build:
   ```bash
   make
   ```

## Running

### Single-channel controllers

1. Choose the appropriate startup script in `iocBoot/iocThorLabs/`:
   | File | Controller | Transport |
   |---|---|---|
   | `AptDC.cmd` | DC servo | local USB serial |
   | `AptDC_ser2net.cmd` | DC servo | remote via ser2net |
   | `AptStepper.cmd` | Stepper | local USB serial |
   | `AptStepper_ser2net.cmd` | Stepper | remote via ser2net |
   | `AptKPZ101_ser2net.cmd` | KPZ101 direct-drive piezo | remote via ser2net |
   | `AptKIM101_ser2net.cmd` | KIM101 4-ch piezo inertial | remote via ser2net |

2. Edit the `.substitutions` file to set PV prefix, motor resolution, etc.

3. Start the IOC:
   ```bash
   cd iocBoot/iocThorLabs
   ../../bin/linux-x86_64/thorlabs st_apt.cmd
   ```

### Multi-channel bay controllers (BSC103, BBD103, etc.)

Use the provided multi-axis templates or create your own:

```
# BSC103 — 3-channel stepper bay controller
dbLoadTemplate("AptBSC103.substitutions")
AptControllerConfig("APT-BSC103", "/dev/ttyUSB0", "stepper", 3, 0.2, 1.0)

# BBD103 — 3-channel DC servo bay controller
dbLoadTemplate("AptBBD103.substitutions")
AptControllerConfig("APT-BBD103", "/dev/ttyUSB0", "dc", 3, 0.2, 1.0)

# Auto-detect number of channels from controller
AptControllerConfig("APT-BSCX", "/dev/ttyUSB0", "stepper", 0, 0.2, 1.0)
```

Example files `AptBSC103.cmd`, `AptBSC103.substitutions`, `AptBBD103.cmd`,
and `AptBBD103.substitutions` are provided in `iocBoot/iocThorLabs/`.

### KIM101 / TIM101 — 4-channel piezo inertial motor controller

the KIM101 exposes all 4 channels over a single USB connection. Each channel
maps to one EPICS motor record `ADDR` (0–3):

```
# KIM101 over ser2net — 4 piezo inertial axes
dbLoadTemplate("AptKPZ101_ser2net.substitutions")
AptControllerConfig("APT-PZ", "hostname:4001", "kim", 4, 0.2, 1.0)
```

Substitutions file example:
```
file "$(MOTOR)/db/motor.db"
{
pattern { PORT,     ADDR, PREFIX,         DESC,             EGU,   MRES, VMAX }
        { "APT-PZ",    0, "KIM:CH1:",  "Piezo inertial 1", count,    1,  500 }
        { "APT-PZ",    1, "KIM:CH2:",  "Piezo inertial 2", count,    1,  500 }
        { "APT-PZ",    2, "KIM:CH3:",  "Piezo inertial 3", count,    1,  500 }
        { "APT-PZ",    3, "KIM:CH4:",  "Piezo inertial 4", count,    1,  500 }
}
```

**Notes:**
- `MRES=1` (one count per step) is recommended; apply real-world calibration
  via the motor record `MRES` or `ERES` field once stage travel is known.
- The controller requires the target channel to be explicitly enabled before
  each move; the driver handles this automatically.
- `home` zeroes the position counter at the current position (no
  physical movement). It does **not** drive to a switch.

### KPZ101 / TPZ001 — single-channel direct-drive piezo controller

```
# KPZ101 over ser2net
dbLoadTemplate("AptKPZ101_ser2net.substitutions")
AptControllerConfig("APT-PZ", "hostname:4001", "piezo", 1, 0.2, 1.0)
```

Position units are 0–32767 (0–100 % of piezo travel).

## IOC Shell Commands

### `AptControllerConfig`

```
AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
```

| Parameter | Description |
|---|---|
| `portName` | EPICS asyn port name (must match `PORT` in substitutions) |
| `devicePath` | `/dev/ttyUSBx` for local serial, or `host:port` for ser2net |
| `motorType` | See table below |
| `numAxes` | `0` = auto-detect, `1` = single-channel (backward compat), `N` = multi-channel |
| `movingPoll` | Polling period in seconds while moving |
| `idlePoll` | Polling period in seconds while idle |

Supported `motorType` values:

| `motorType` | Protocol family | Typical controllers |
|---|---|---|
| `"dc"` | `MOT_*` (0x04xx) | KDC001, KDC101, TDC001, BBD10x |
| `"stepper"` | `MOT_*` (0x04xx) | KST101, TST001, BSC10x |
| `"piezo"` | `PZ_*` (0x06xx) | KPZ101, TPZ001 |
| `"kim"` / `"pzmot"` / `"tim"` | `PZMOT_*` (0x08xx) | KIM101, TIM101 |

**Protocol notes:**
- When `numAxes=1` (or auto-detected as 1), messages are sent to `dest=0x50`
  (Generic USB), which is required for single-channel USB units.
- When `numAxes>1` with `motorType="dc"` or `"stepper"`, `dest=0x11`
  (Rack/motherboard) is used and the APT `ChanIdent` field routes each
  message to the correct bay.
- For `motorType="kim"` / `"pzmot"`, `dest=0x50` is always used regardless
  of `numAxes`; the channel is selected via per-message sub-parameters in
  the PZMOT protocol.

### `AptSetVelParams`

Set velocity parameters directly in APT internal units (bypasses motor record
unit conversion):

```
AptSetVelParams(portName, axisNum, minVelocity, acceleration, maxVelocity)
```

| Parameter | Description |
|---|---|
| `portName` | EPICS asyn port name |
| `axisNum` | Axis index (0-based, matches `ADDR` in substitutions) |
| `minVelocity` | Minimum velocity in APT internal units |
| `acceleration` | Acceleration in APT internal units |
| `maxVelocity` | Maximum velocity in APT internal units |

Example (KDC101 driving a Z8xx stage, 2 mm/s):
```
AptSetVelParams("APT-DC1", 0, 0, 263844, 772981)
```

### Debug level

```
var drvAptDebug 2    # 0=off, 1=errors, 2=moves+poll, 3=APT trace, 4=raw hex
```

## EPICS Motor Record Addressing

Each APT channel maps to a motor record `ADDR` field:

| ADDR | APT channel | Bay (rack controllers) |
|---|---|---|
| 0 | 1 | Bay 0 |
| 1 | 2 | Bay 1 |
| 2 | 3 | Bay 2 |

## Serial Device Setup

On Linux, Thorlabs controllers appear as `/dev/ttyUSBx` devices
when the FTDI USB driver is loaded (typically automatic).

### Persistent device names via udev

```
# /etc/udev/rules.d/99-thorlabs.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="83xxxxxx", SYMLINK+="thorlabs_dc0"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="80xxxxxx", SYMLINK+="thorlabs_step0"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="70xxxxxx", SYMLINK+="thorlabs_bsc103"
```

## Remote Access via ser2net

The driver supports connecting to controllers attached to remote hosts
via ser2net — a serial-to-network proxy.

Example ser2net v4 configuration (on the remote host):
```yaml
connection: &thorlabs-bsc103
  accepter: tcp,4001
  connector: serialdev,/dev/ttyUSB0,115200n81,local,rtscts
  options:
    kickolduser: true
```

Then in the IOC startup:
```
AptControllerConfig("APT-BSC103", "hostname:4001", "stepper", 3, 0.2, 1.0)
```

## Communication Parameters

Per the Thorlabs APT protocol specification:
- 115200 baud, 8 data bits, 1 stop bit, no parity
- RTS/CTS hardware flow control

## Author

Andrea Michelotti — <andrea.michelotti@infn.it>  
INFN Laboratori Nazionali di Frascati (LNF)
