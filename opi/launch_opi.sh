#!/bin/bash
# ============================================================================
# Thorlabs APT Motor - Phoebus OPI Launcher
# ============================================================================
#
# Usage:
#   ./launch_opi.sh                         # default: launcher (4-ch piezo overview)
#   ./launch_opi.sh motor <prefix> <motor>  # single motor:  P=<prefix> M=<motor>
#   ./launch_opi.sh piezo <prefix> <motor>  # single piezo:  P=<prefix> M=<motor>
#   ./launch_opi.sh launcher [prefix]       # launcher view: P=<prefix>
#
# Examples:
#   ./launch_opi.sh                         # opens launcher with P=Motor:
#   ./launch_opi.sh launcher Motor:         # opens launcher with P=Motor:
#   ./launch_opi.sh motor Motor: DC1        # opens motor display for DC1
#   ./launch_opi.sh piezo Motor: PZ1        # opens piezo display for PZ1
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-launcher}"
case "$MODE" in
    motor)
        OPI_FILE="${SCRIPT_DIR}/ThorlabsMotor.bob"
        P="${2:-Motor:}"
        M="${3:-DC1}"
        MACROS="P=${P}&M=${M}"
        ;;
    piezo)
        OPI_FILE="${SCRIPT_DIR}/ThorlabsPiezo.bob"
        P="${2:-Motor:}"
        M="${3:-PZ1}"
        MACROS="P=${P}&M=${M}"
        ;;
    launcher|*)
        OPI_FILE="${SCRIPT_DIR}/ThorlabsLauncher.bob"
        P="${2:-Motor:}"
        MACROS="P=${P}"
        ;;
esac

# Try to find Phoebus
if command -v phoebus &>/dev/null; then
    PHOEBUS_CMD="phoebus"
elif [ -n "$PHOEBUS_HOME" ]; then
    PHOEBUS_CMD="${PHOEBUS_HOME}/phoebus.sh"
elif [ -x "/opt/phoebus/phoebus.sh" ]; then
    PHOEBUS_CMD="/opt/phoebus/phoebus.sh"
else
    echo "ERROR: Cannot find Phoebus. Set PHOEBUS_HOME or add phoebus to PATH."
    echo ""
    echo "To open manually, use:"
    echo "  phoebus -resource '${OPI_FILE}?P=${P}&M=${M}'"
    exit 1
fi

echo "Opening Thorlabs display: ${OPI_FILE##*/}  (${MACROS})"

exec "${PHOEBUS_CMD}" -resource "${OPI_FILE}?${MACROS}" &
