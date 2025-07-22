#!/bin/bash
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -eu

# CAN Interface Script
# Usage: scripts/set_zero.sh <CAN_IF> [CAN_ID] [-all]

# Function to display usage
usage() {
    echo "Usage: $0 <CAN_IF> [CAN_ID] [-all]"
    echo "  CAN_IF: CAN interface name (e.g., can0)"
    echo "  CAN_ID: CAN ID in hex format (e.g., 00x) - not needed with -all"
    echo "  -all: Send to all IDs from 001 to 008"
    echo ""
    echo "Examples:"
    echo "  $0 can0 001"
    echo "  $0 can0 -all"
    exit 1
}

# Function to check if CAN interface is up and get baudrate
check_can_interface() {
    local interface=$1

    # Check if interface exists and is up
    if ! ip link show "$interface" &>/dev/null; then
        echo "Error: CAN interface $interface does not exist"
        return 1
    fi

    # Check if interface is up
    local state
    state=$(ip link show "$interface" | grep -o "state [A-Z]*" | cut -d' ' -f2)
    if [ "$state" != "UP" ]; then
        echo "Error: CAN interface $interface is not UP (current state: $state)"
        return 1
    fi

    echo "CAN interface $interface is UP"

    # Try to get baudrate information
    if command -v ethtool &>/dev/null; then
        local baudrate
        baudrate=$(ethtool "$interface" 2>/dev/null | grep -i speed | cut -d: -f2 | tr -d ' ')
        if [ -n "$baudrate" ]; then
            echo "Baudrate: $baudrate"
        fi
    fi

    # Alternative method using ip command
    local bitrate
    bitrate=$(ip -details link show "$interface" 2>/dev/null | grep -o "bitrate [0-9]*" | cut -d' ' -f2)
    if [ -n "$bitrate" ]; then
        echo "Bitrate: ${bitrate} bps"
    fi

    return 0
}

# Function to send CAN messages for a single ID
send_can_messages() {
    local CAN_ID=$1
    local CAN_IF=$2

    echo "Sending CAN messages for ID: $CAN_ID on interface: $CAN_IF"

    # Send first disablemessage
    echo "Sending: cansend $CAN_IF ${CAN_ID}#FFFFFFFFFFFFFFFD"
    cansend "$CAN_IF" "${CAN_ID}#FFFFFFFFFFFFFFFD"

    sleep 0.1

    # Send second set zero message
    echo "Sending: cansend $CAN_IF ${CAN_ID}#FFFFFFFFFFFFFFFE"
    cansend "$CAN_IF" "${CAN_ID}#FFFFFFFFFFFFFFFE"

    sleep 0.1

    # Send third disable message
    echo "Sending: cansend $CAN_IF ${CAN_ID}#FFFFFFFFFFFFFFFD"
    cansend "$CAN_IF" "${CAN_ID}#FFFFFFFFFFFFFFFD"

    sleep 0.1

    echo "Messages sent for ID: $CAN_ID"
    echo ""
}

# Main script logic
main() {
    # Check for minimum arguments
    if [ $# -lt 1 ]; then
        usage
    fi

    local CAN_IF=$1
    local CAN_ID=""
    local all_flag=false

    # Check for -all flag
    if [ "$2" = "-all" ]; then
        all_flag=true
    else
        CAN_ID=$2
        if [ "$3" = "-all" ]; then
            all_flag=true
        fi
    fi

    # Validate CAN_IF
    if [ -z "$CAN_IF" ]; then
        usage
    fi

    # Validate CAN_ID only if -all flag is not set
    if [ "$all_flag" = false ] && [ -z "$CAN_ID" ]; then
        echo "Error: CAN_ID is required when -all flag is not used"
        usage
    fi

    # Check if cansend command is available
    if ! command -v cansend &>/dev/null; then
        echo "Error: cansend command not found. Please install can-utils package."
        exit 1
    fi

    # Check CAN interface status
    if ! check_can_interface "$CAN_IF"; then
        exit 1
    fi

    echo ""

    # Execute based on flags
    if [ "$all_flag" = true ]; then
        echo "Sending set zero messages to all motor with CAN IDs from 001 to 008"
        echo "=========================================="
        for i in {1..8}; do
            local padded_id
            padded_id=$(printf "%03d" "$i")
            send_can_messages "$padded_id" "$CAN_IF"
        done
    else
        send_can_messages "$CAN_ID" "$CAN_IF"
    fi

    echo "Set zero completed."
}

# Run main function with all arguments
main "$@"
