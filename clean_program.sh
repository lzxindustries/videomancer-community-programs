#!/bin/bash
# Videomancer Community Programs - Single Program Clean Script
# Copyright (C) 2025 LZX Industries LLC
# SPDX-License-Identifier: GPL-3.0-only
#
# Removes build artifacts and packaged output for a single program.
# Usage: ./clean_program.sh <vendor> <program>
# Example: ./clean_program.sh boneoh rgb_bit_crush

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

VIDEOMANCER_BUILD_DIR="${SCRIPT_DIR}/build/programs"
VIDEOMANCER_OUT_DIR="${SCRIPT_DIR}/out"

if [ $# -ne 2 ]; then
    echo -e "${RED}ERROR: Expected exactly 2 arguments${NC}"
    echo -e "${YELLOW}Usage:   $0 <vendor> <program>${NC}"
    echo -e "${YELLOW}Example: $0 boneoh rgb_bit_crush${NC}"
    exit 1
fi

VENDOR="$1"
PROGRAM="$2"

echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Cleaning: ${VENDOR}/${PROGRAM}${NC}"
echo -e "${BLUE}====================================${NC}"
echo ""

# Clean build artifacts
BUILD_DIR="${VIDEOMANCER_BUILD_DIR}/${VENDOR}/${PROGRAM}"
if [ -d "${BUILD_DIR}" ]; then
    echo -e "${CYAN}Removing build artifacts: ${BUILD_DIR}${NC}"
    rm -rf "${BUILD_DIR}"
    echo -e "${GREEN}✓ Removed${NC}"
else
    echo -e "${YELLOW}  Build directory not found (already clean): ${BUILD_DIR}${NC}"
fi

echo ""

# Clean .vmprog output across all hardware variant directories
VMPROG_COUNT=0
if [ -d "${VIDEOMANCER_OUT_DIR}" ]; then
    echo -e "${CYAN}Removing packaged output for ${VENDOR}/${PROGRAM}...${NC}"
    while IFS= read -r -d '' vmprog; do
        rm -f "$vmprog"
        echo -e "${GREEN}✓ Removed: ${vmprog}${NC}"
        VMPROG_COUNT=$((VMPROG_COUNT + 1))
    done < <(find "${VIDEOMANCER_OUT_DIR}" -path "*/${VENDOR}/${PROGRAM}.vmprog" -print0 2>/dev/null)

    if [ "$VMPROG_COUNT" -eq 0 ]; then
        echo -e "${YELLOW}  No .vmprog files found (already clean)${NC}"
    fi
else
    echo -e "${YELLOW}  Output directory not found: ${VIDEOMANCER_OUT_DIR}${NC}"
fi

echo ""
echo -e "${GREEN}====================================${NC}"
echo -e "${GREEN}Clean complete for ${VENDOR}/${PROGRAM}${NC}"
echo -e "${GREEN}====================================${NC}"
