#!/bin/bash
# Videomancer Community Programs - Single Program Build Script
# Copyright (C) 2025 LZX Industries LLC
# SPDX-License-Identifier: GPL-3.0-only
#
# Convenience wrapper around build_programs.sh for building a single program.
# Usage: ./build_program.sh <vendor> <program>
# Example: ./build_program.sh boneoh rgb_bit_crush

# Colors for output
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ $# -ne 2 ]; then
    echo -e "${RED}ERROR: Expected exactly 2 arguments${NC}"
    echo -e "${YELLOW}Usage:   $0 <vendor> <program>${NC}"
    echo -e "${YELLOW}Example: $0 boneoh rgb_bit_crush${NC}"
    exit 1
fi

exec "${SCRIPT_DIR}/build_programs.sh" "$1" "$2"
