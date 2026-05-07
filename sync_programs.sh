#!/bin/bash
#
# sync_programs.sh
#
# Copy newer files and any new files/folders from
#   /Users/peterappleby/videomancer-community-programs/programs/boneoh
# into
#   /Users/peterappleby/videomancer-sdk-main/programs/boneoh/
#
# Behavior:
#   - Files that are NEWER in boneoh overwrite the older copies in programs.
#   - Files/folders that are NEW in boneoh are copied over.
#   - Files/folders that exist ONLY in programs are left alone.
#   - .DS_Store and other macOS junk are excluded.
#
# Usage:
#   ./sync_programs.sh [options]
#   ./sync_programs.sh --dry-run
#
# Options:
#   --dry-run    Show what would be copied without actually copying
#   --help       Show this help message
#
# Requires: rsync (preinstalled on macOS)
#
set -euo pipefail

# Default values
SRC="/Users/peterappleby/videomancer-community-programs/programs/boneoh/"
DST="/Users/peterappleby/videomancer-sdk/programs/"
BACKUP_BASE="/Volumes/Elektron 1 TB/VM-Backup"
DRY_RUN=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --dry-run    Show what would be copied without actually copying"
            echo "  --help       Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
    esac
done

# Create timestamped backup directory
TIMESTAMP=$(date +"%Y.%m.%d.%H.%M")
BACKUP_DIR="${BACKUP_BASE}/boneoh.${TIMESTAMP}"

echo "Creating backup in: $BACKUP_DIR"
if [ "$DRY_RUN" = true ]; then
    echo "(dry run) Would create backup in: $BACKUP_DIR"
else
    mkdir -p "$BACKUP_DIR"
fi

# Copy all files from SRC to backup directory
echo "Backing up files from: $SRC"
if [ "$DRY_RUN" = true ]; then
    echo "(dry run) Would back up files from: $SRC"
else
    cp -r "$SRC"/* "$BACKUP_DIR"/ 2>/dev/null || true
fi

# Make the backup directory and all its contents read-only
echo "Setting read-only permissions on backup directory"
if [ "$DRY_RUN" = true ]; then
    echo "(dry run) Would set read-only permissions on backup directory"
else
    chmod -R a-w "$BACKUP_DIR"
fi

# Sanity checks.
if [ ! -d "$SRC" ]; then
    echo "sync_programs.sh: source folder not found: $SRC" >&2
    exit 1
fi
if [ ! -d "$DST" ]; then
    echo "sync_programs.sh: destination folder not found: $DST" >&2
    exit 1
fi

echo "Syncing:"
echo "  from: $SRC"
echo "  to:   $DST"
if [ "$DRY_RUN" = true ]; then
    echo "  (dry run mode - no files will be copied)"
fi
echo

# Build rsync command with optional dry-run flag
RSYNC_CMD="rsync -avhu"
if [ "$DRY_RUN" = true ]; then
    RSYNC_CMD="$RSYNC_CMD --dry-run"
fi

# Add exclusion patterns
EXCLUDE_PATTERNS=(
    "--exclude=.DS_Store"
    "--exclude=._*"
    "--exclude=.AppleDouble"
    "--exclude=.Spotlight-V100"
    "--exclude=.Trashes"
)

# Execute rsync command
if [ "$DRY_RUN" = true ]; then
    echo "(dry run) Would execute: $RSYNC_CMD ${EXCLUDE_PATTERNS[*]} $SRC $DST"
else
    eval "$RSYNC_CMD ${EXCLUDE_PATTERNS[*]} $SRC $DST"
fi

echo
echo "Done."
