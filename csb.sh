#!/bin/bash

#
# csb.sh - Clean, Sync, Build Script
#
# This script runs three sequential operations:
# 1. clean_programs.sh - Cleans the programs directory
# 2. sync_programs.sh - Syncs programs from community to SDK
# 3. build_programs.sh - Builds the programs
#
# Output is displayed on console and saved to a timestamped file in:
# /Users/peterappleby/videomancer-community-programs/programs/boneoh/

set -euo pipefail

# Get current timestamp for filename
TIMESTAMP=$(date +"%Y.%m.%d.%H.%M")
OUTPUT_FILE="/Users/peterappleby/videomancer-community-programs/programs/boneoh/Clean-Sync-Build.${TIMESTAMP}.txt"

echo "Running Clean-Sync-Build sequence..."
echo "Output will be saved to: $OUTPUT_FILE"
echo

# Create output file with header
{
    echo "Clean-Sync-Build Sequence Log"
    echo "Timestamp: $(date)"
    echo "========================================="
    echo
} > "$OUTPUT_FILE"

# Function to run a command and append its output to both console and file
run_with_logging() {
    local cmd="$1"
    local description="$2"
    
    echo "=== $description ===" | tee -a "$OUTPUT_FILE"
    echo "Command: $cmd" | tee -a "$OUTPUT_FILE"
    echo | tee -a "$OUTPUT_FILE"
    
    # Execute command and capture output
    eval "$cmd" 2>&1 | tee -a "$OUTPUT_FILE"
    
    echo | tee -a "$OUTPUT_FILE"
}

# Run the three scripts in sequence
run_with_logging "./clean_programs.sh" "CLEAN PROGRAMS"
run_with_logging "./sync_programs.sh" "SYNC PROGRAMS" 
run_with_logging "./build_programs.sh" "BUILD PROGRAMS"

echo "Clean-Sync-Build sequence completed."
echo "Full log saved to: $OUTPUT_FILE"