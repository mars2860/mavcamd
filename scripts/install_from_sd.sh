#!/bin/sh

# Determine the absolute path to the directory where this script is located on the SD card
SRC_DIR="$(dirname "$(readlink -f "$0")")"

echo "=== OpenIPC Target Installation Script ==="
echo "Source directory: $SRC_DIR"

# Helper function to safely copy files and apply configurations
deploy_file() {
    local src="$1"
    local dest="$2"
    local perm="$3"

    if [ -f "$src" ]; then
        printf "Deploying %s -> %s... " "$(basename "$src")" "$dest"
        # Copy the file to target destination
        cp -f "$src" "$dest"
        if [ $? -eq 0 ]; then
            # Apply execution or access permissions if provided
            if [ -n "$perm" ]; then
                chmod "$perm" "$dest"
            fi
            echo "OK"
        else
            echo "FAILED to copy"
        fi
    else
        echo "WARNING: Source file $(basename "$src") not found in SD folder, skipping."
    fi
}

# 1. Remount the filesystem to read-write mode in case it is currently read-only
mount -o remount,rw /

echo "------------------------------------------"
echo "Stopping active processes if running..."

# 2. Check and stop mavcamd daemon
if pidof mavcamd > /dev/null; then
    echo "mavcamd process is currently running. Stopping it..."
    if [ -x /etc/init.d/S99mavcamd ]; then
        /etc/init.d/S99mavcamd stop
    else
        killall -q mavcamd
    fi
    sleep 1
else
    echo "mavcamd is not running."
fi

# 3. Check and stop mavlink-routerd daemon
if pidof mavlink-routerd > /dev/null; then
    echo "mavlink-routerd process is currently running. Stopping it..."
    if [ -x /etc/init.d/S97mavlink ]; then
        /etc/init.d/S97mavlink stop
    else
        killall -q mavlink-routerd
    fi
    sleep 1
else
    echo "mavlink-routerd is not running."
fi

echo "------------------------------------------"
echo "Starting deployment of new files..."

# 4. Deploy core executable binary assets
deploy_file "$SRC_DIR/mavcamd" "/usr/sbin/mavcamd" "755"
deploy_file "$SRC_DIR/mavlink-routerd" "/usr/sbin/mavlink-routerd" "755"

# 5. Deploy system configuration setups
deploy_file "$SRC_DIR/mavcam.json" "/etc/mavcam.json" "644"
deploy_file "$SRC_DIR/mavlink.conf" "/etc/mavlink.conf" "644"

# 6. Deploy SysVinit automated initialization system startup scripts
deploy_file "$SRC_DIR/S97mavlink" "/etc/init.d/S97mavlink" "755"
deploy_file "$SRC_DIR/S99mavcamd" "/etc/init.d/S99mavcamd" "755"

echo "------------------------------------------"
echo "File deployment finished. Activating processes in sequence..."

# 7. Start services in the correct sequence (packet router infrastructure first)
if [ -x /etc/init.d/S97mavlink ]; then
    /etc/init.d/S97mavlink start
    sleep 1
fi

if [ -x /etc/init.d/S99mavcamd ]; then
    /etc/init.d/S99mavcamd start
fi

echo "=== Installation Successful! ==="
