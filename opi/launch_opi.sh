#!/bin/bash
# ============================================================================
# Thorlabs APT Motor - Phoebus OPI Launcher
# ============================================================================
#
# Usage:
#   ./launch_opi.sh                         # default: P=Motor: M=DC1
#   ./launch_opi.sh <prefix> <motor>        # custom:  P=<prefix> M=<motor>
#
# Examples:
#   ./launch_opi.sh Motor: DC1
#   ./launch_opi.sh Motor: Step1
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPI_FILE="${SCRIPT_DIR}/ThorlabsMotor.bob"

P="${1:-Motor:}"
M="${2:-DC1}"

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

echo "Opening Thorlabs Motor display:  P=${P}  M=${M}"
echo "  OPI: ${OPI_FILE}"

exec "${PHOEBUS_CMD}" -resource "${OPI_FILE}?P=${P}&M=${M}" &
