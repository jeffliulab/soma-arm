#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/video0"
POWER_LINE_HZ="60"
EXPOSURE_ABSOLUTE="156"
WHITE_BALANCE_TEMPERATURE=""
FOCUS_ABSOLUTE=""
DUMP_ONLY=0

usage() {
    cat <<'EOF'
Usage:
  scripts/lock_c922_params.sh [options]

Options:
  --device /dev/video0              V4L2 device path. Default: first /dev/video*
  --power-line-hz 50|60|auto|off    Anti-flicker preset. Default: 60
  --exposure N                      Manual exposure_absolute value. Default: 156
  --white-balance-temperature N     Optional manual white balance temperature
  --focus N                         Optional manual focus_absolute value
  --dump-only                       Only print current camera controls
  -h, --help                        Show this help

Notes:
  - Default behavior is tuned for an indoor U.S. desk setup:
    power line = 60 Hz, manual exposure, auto-priority off,
    auto white balance off, auto focus off.
  - exposure_absolute=156 is close to a 1/60 s exposure window,
    which is a reasonable starting point for reducing LED flicker.
EOF
}

detect_default_device() {
    local candidate
    for candidate in /dev/video*; do
        [[ -e "$candidate" ]] || continue
        echo "$candidate"
        return 0
    done
    echo "/dev/video0"
}

bool_to_v4l2() {
    case "${1,,}" in
        on|true|1|yes)
            echo 1
            ;;
        off|false|0|no)
            echo 0
            ;;
        *)
            echo "Unsupported boolean value: $1" >&2
            exit 1
            ;;
    esac
}

normalize_power_line_label() {
    case "${1,,}" in
        50|50hz|50-hz)
            echo "50 hz"
            ;;
        60|60hz|60-hz)
            echo "60 hz"
            ;;
        auto)
            echo "auto"
            ;;
        off|disabled|disable)
            echo "disabled"
            ;;
        *)
            echo "Unsupported power-line preset: $1" >&2
            exit 1
            ;;
    esac
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --power-line-hz)
            POWER_LINE_HZ="$2"
            shift 2
            ;;
        --exposure)
            EXPOSURE_ABSOLUTE="$2"
            shift 2
            ;;
        --white-balance-temperature)
            WHITE_BALANCE_TEMPERATURE="$2"
            shift 2
            ;;
        --focus)
            FOCUS_ABSOLUTE="$2"
            shift 2
            ;;
        --dump-only)
            DUMP_ONLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ "$DEVICE" == "/dev/video0" && ! -e "$DEVICE" ]]; then
    DEVICE="$(detect_default_device)"
fi

if [[ ! -e "$DEVICE" ]]; then
    echo "Camera device not found: $DEVICE" >&2
    echo "Hint: run \`ls /dev/video*\` first and make sure the C922 is attached to WSL." >&2
    exit 1
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
    echo "v4l2-ctl is not installed. Install \`v4l-utils\` first." >&2
    exit 1
fi

CTRL_OUTPUT="$(
    {
        v4l2-ctl -d "$DEVICE" --list-ctrls-menus 2>/dev/null || true
        v4l2-ctl -d "$DEVICE" --list-ctrls 2>/dev/null || true
    }
)"

control_exists() {
    local control="$1"
    grep -q "^${control} " <<<"$CTRL_OUTPUT"
}

menu_index() {
    local control="$1"
    local label="$2"

    awk -v control="$control" -v label="$label" '
        BEGIN {
            want = tolower(label)
        }
        $1 == control {
            active = 1
            next
        }
        active && /^[^[:space:]]/ {
            active = 0
        }
        active {
            lowered = tolower($0)
            if (index(lowered, want) > 0) {
                value = $1
                sub(/:$/, "", value)
                print value
                exit
            }
        }
    ' <<<"$CTRL_OUTPUT"
}

set_control() {
    local control="$1"
    local value="$2"
    local note="$3"

    if ! control_exists "$control"; then
        echo "Skipping ${control}: this device does not expose it."
        return 0
    fi

    echo "Setting ${control}=${value} (${note})"
    v4l2-ctl -d "$DEVICE" -c "${control}=${value}"
}

set_menu_control() {
    local control="$1"
    local label="$2"
    local index

    if ! control_exists "$control"; then
        echo "Skipping ${control}: this device does not expose it."
        return 0
    fi

    index="$(menu_index "$control" "$label")"
    if [[ -z "$index" ]]; then
        echo "Skipping ${control}: could not find menu entry matching '${label}'." >&2
        return 0
    fi

    echo "Setting ${control}=${index} (${label})"
    v4l2-ctl -d "$DEVICE" -c "${control}=${index}"
}

print_relevant_controls() {
    local control

    echo
    echo "Current relevant controls for ${DEVICE}:"
    for control in \
        power_line_frequency \
        exposure_auto \
        exposure_auto_priority \
        exposure_absolute \
        white_balance_temperature_auto \
        white_balance_temperature \
        focus_auto \
        focus_absolute
    do
        if control_exists "$control"; then
            v4l2-ctl -d "$DEVICE" -C "$control" 2>/dev/null || true
        fi
    done
}

echo "Using camera device: ${DEVICE}"

if (( DUMP_ONLY )); then
    v4l2-ctl -d "$DEVICE" --list-ctrls-menus
    exit 0
fi

set_menu_control "power_line_frequency" "$(normalize_power_line_label "$POWER_LINE_HZ")"
set_menu_control "exposure_auto" "manual mode"
set_control "exposure_auto_priority" "$(bool_to_v4l2 off)" "keep FPS stable"
set_control "exposure_absolute" "$EXPOSURE_ABSOLUTE" "manual exposure baseline"
set_control "white_balance_temperature_auto" "$(bool_to_v4l2 off)" "freeze white balance"
if [[ -n "$WHITE_BALANCE_TEMPERATURE" ]]; then
    set_control "white_balance_temperature" "$WHITE_BALANCE_TEMPERATURE" "manual white balance"
fi
set_control "focus_auto" "$(bool_to_v4l2 off)" "freeze focus"
if [[ -n "$FOCUS_ABSOLUTE" ]]; then
    set_control "focus_absolute" "$FOCUS_ABSOLUTE" "manual focus"
fi

print_relevant_controls

cat <<EOF

Next:
  1. Re-run the preview:
     python3 scripts/preview_camera_linux.py --width 1280 --height 720 --fps 60 --mjpeg
  2. If the image is still too dark or bright, try:
     scripts/lock_c922_params.sh --device ${DEVICE} --exposure 180
     scripts/lock_c922_params.sh --device ${DEVICE} --exposure 120
EOF
