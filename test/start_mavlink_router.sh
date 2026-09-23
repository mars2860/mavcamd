#!/bin/bash

# Check if the script is running with sudo/root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "sudo $0"
  exit 1
fi

echo "Starting mavlink-routerd..."
# Run the mavlink-routerd command with the specified configuration file
mavlink-routerd -c ./router.conf