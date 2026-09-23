#!/bin/bash

# Check if the script is running with sudo/root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "sudo $0"
  exit 1
fi

echo "Starting Python web server on port 80..."
# Run the server
python3 -m http.server 80