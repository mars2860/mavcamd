# mavcamd

`mavcamd` is a lightweight, high-performance daemon designed for **OpenIPC-based IP cameras** (such as boards powered by SigmaStar SSC338Q, HiSilicon, or Goke processors). It acts as a bridge between the **MAVLink Camera Protocol v2** and the camera's internal video encoder/REST API management layers.

The program runs as a **UDP client** and immediately starts broadcasting MAVLink Camera `HEARTBEAT` messages (acting as a MAVLink camera component) upon startup. It then waits for corresponding MAVLink control messages coming from the Ground Control Station (GCS) and the autopilot. 

To bridge these components, `mavlink-routerd` is used to intercept and forward MAVLink packets from the autopilot and GCS directly to the UDP address monitored by `mavcamd`. In response to these incoming commands, `mavcamd` invokes the designated **REST API functions** specified inside its `mavcam.json` configuration profile, seamlessly exposing hardware actions like photo triggering, video streaming, or storage formatting.

---

## Features

- **Partially implements** the MAVLink Camera Protocol v2 (Video Streams, Video Recording, Photos, Storages).
- Dynamically parses video encoding configurations from the system state layer **only when utilizing the OpenIPC Waybeam streamer**.
- Handles automated storage operations (e.g., recursive payload folder clearing on format commands).

---

## Project Directory Layout (Testing Environment)

All configuration components, placeholder stubs, and orchestration shell shortcuts allocated for validating the software operation local run routines reside inside the `test/` directory.

---

## Compilation

### 1. Compiling for Local Native Testing (Linux PC x86_64)

To build and validate logic operations directly on a development computer, use the following local `cmake` sequence:

```bash
mkdir -p build && cd build
cmake ..
make
```

### 2. Cross-Compiling for OpenIPC Ecosystem (Buildroot Setup)

Before triggering the platform compilation sequence, you must add the `mavcamd` package integration recipe into the source tree of the **openipc-firmware** repository in accordance with the official OpenIPC Buildroot packaging documentation guidelines.

Once the package environment definition rules are integrated, execute the following commands to build a target binary matching your exact target processor profile (e.g., **SigmaStar SSC338Q**):

```bash
make CHIP=sigmastar BOARD=ssc338q_ultimate br-mavcamd-rebuild
```

---

## Installation on OpenIPC Target Hardware

### 1. Deploying mavcamd Binary & Config
Once successfully compiled inside the Buildroot tree, deploy the generated runtime binary assets manually across the network interface over SSH to your operating target hardware (e.g., **Wifilink2** board):

```bash
# Push executable bin to system binary folder
scp openipc-firmware/output/build/mavcamd-1.0.0/mavcamd root@<CAMERA_IP>:/usr/sbin/mavcamd

# Deploy initialization automation scripts
scp <your_source_path>/mavcamd/scripts/mavcam.json root@<CAMERA_IP>:/etc/mavcam.json
```

### 2. Setting Up Initialization Scripts (Autostart)
To ensure the daemon starts automatically every time the camera boots up, copy the initialization script **`S99mavcamd`** into the target system's **`/etc/init.d/`** folder and make it executable:

```bash
# Copy the startup manager script across the network link
scp <your_source_path>/mavcamd/scripts/S99mavcamd root@<CAMERA_IP>:/etc/init.d/S99mavcamd
```

### 3. Installing and Configuring mavlink-routerd
To route telemetry data properly between the autopilot, GCS, and `mavcamd`, you must also ensure that **`mavlink-routerd`** is installed on the OpenIPC camera system.

Create a configuration file named `mavlink.conf` for `mavlink-routerd` to define the routing endpoints. Below is a production template example:

```ini
[General]
ReportEmitters=false

# Autopilot serial connection link via UART (example: /dev/ttyS1 at 57600 baud)
[UartEndpoint autopilot]
Device=/dev/ttyS1
Baud=57600

# Ground Control Station network connection link (example: UDP broadcast to port 14550)
[UdpEndpoint gcs]
Mode=Normal
Address=0.0.0.0
Port=14550

# Local routing redirection pointing straight to mavcamd daemon network listener port
[UdpEndpoint mavcamd]
Mode=Server
Address=127.0.0.1
Port=14553
```

Copy this configuration file into the **`/etc/`** directory on your target hardware camera filesystem:
```bash
scp <your_source_path>/mavcamd/scripts/mavlink.conf root@<CAMERA_IP>:/etc/mavlink.conf
```

---

## Command Line Interface (CLI) Arguments

```text
MAVLink Camera Protocol Daemon for OpenIPC

Usage: mavcamd [options]

Options:
  -a, --address <ip:port>   IP address and UDP port for mavlink traffic
  -c, --config  <file>      Path to MAVLink Camera configuration JSON file
  -t, --temp  <file>        Inject SoC temperature into telemetry(HiSilicon and SigmaStart supported) via RAW_IMU mavlink message
  -h, --help                Show this help message

For example with logging: mavcamd -a 127.0.0.1:14553 > /tmp/mavcamd.log 2>&1
```

---

## JSON Configuration Profile Specification (`mavcam.json`)

The application configuration includes native camera telemetry parameters, storage device blocks, REST API routings, and optional predefined video stream entries.

### Full Profile Example with Predefined Streams
```json
{
  "camera_information": {
    "component_id": 100,
    "vendor_name": "Runcam",
    "model_name": "Wifilink2",
    "firmware_version": 1,
    "focal_length": 4,
    "sensor_size_h": 5.6028,
    "sensor_size_v": 3.1784,
    "resolution_h": 3864,
    "resolution_v": 2192,
    "lens_id": 0,
    "flags": 7,
    "cam_definition_version": 0,
    "cam_definition_uri": "",
    "gimbal_device_id": 1,
    "camera_device_id": 0
  },
  "video_streams": [
    {
      "stream_id": 1,
      "type": 1,
      "flags": 1,
      "framerate": 60.0,
      "resolution_h": 1920,
      "resolution_v": 1080,
      "bitrate": 26000,
      "rotation": 0,
      "hfov": 70,
      "encoding": 2,
      "camera_device_id": 0,
      "name": "main_1080p_h265",
      "uri": "5600"
    },
    {
      "stream_id": 2,
      "type": 1,
      "flags": 0,
      "framerate": 30.0,
      "resolution_h": 1280,
      "resolution_v": 720,
      "bitrate": 8192,
      "rotation": 0,
      "hfov": 70,
      "encoding": 2,
      "camera_device_id": 0,
      "name": "sub_720p_h265",
      "uri": "5610"
    }
  ],
  "storages": [
    {
      "storage_id": 2,
      "type": 3,
      "name": "SDCard",
      "dcim": "/mnt/mmcblk0p1/DCIM3",
      "storage_usage": 15,
      "read_speed": 140,
      "write_speed": 100
    }
  ],
  "rest_api": {
    "base_url": "http://127.0.0.1/api/v1",
    "video_start": "/record/start",
    "video_stop": "/record/stop",
    "video_status": "/record/status",
    "photo_capture": "/snapshot.jpg"
  }
}
```

### Profile Example Leveraging Automated Waybeam Parsing
If the `"video_streams"` array block is **omitted** from the configuration profile while `"venc": "waybeam"` is specified in the `"rest_api"` scope, the daemon triggers an automated parsing algorithm to discover stream setups directly from `/etc/waybeam.json` on the fly.

```json
{
  "camera_information": {
    "component_id": 100,
    "vendor_name": "Runcam",
    "model_name": "Wifilink2",
    "firmware_version": 1,
    "focal_length": 4,
    "sensor_size_h": 5.6028,
    "sensor_size_v": 3.1784,
    "resolution_h": 3864,
    "resolution_v": 2192,
    "lens_id": 0,
    "flags": 7,
    "gimbal_device_id": 1,
    "camera_device_id": 0
  },
  "storages": [
    {
      "storage_id": 1,
      "type": 3,
      "name": "SDCard",
      "dcim": "/mnt/mmcblk0p1/DCIM3",
      "storage_usage": 15,
      "read_speed": 140,
      "write_speed": 100
    }
  ],
  "rest_api": {
    "venc": "waybeam",
    "base_url": "http://127.0.0.1/api/v1",
    "video_start": "/record/start",
    "video_stop": "/record/stop",
    "video_status": "/record/status",
    "photo_capture": "/snapshot.jpg"
  }
}
```

---

## Local Verification & Simulation Run (PC to PX4 Autopilot)

To perform functional validation on a local Linux development workstation with a physical **PX4 Flight Controller** attached via a hardware USB interface connection (`/dev/ttyACM0`):

1. **Navigate to the Test Directory**:
   ```bash
   cd test
   ```

2. **Spin up Mock Subsystems**:
   Open a separate terminal shell and execute the script simulating internal camera engine HTTP responses:
   ```bash
   ./start_server.sh
   ```

3. **Launch the MAVLink Router Process**:
   Route telemetry back and forth split across local test instances and the serial USB physical interface node link layer:
   ```bash
   ./start_mavlink_router.sh
   ```

4. **Execute `mavcamd` with the Target Test Configuration**:
   Run the local compiled software executable instance referencing the chosen profile path parameter targets and targeting the local router interface:
   ```bash
   ../build/mavcamd -c mavcam_test_conf1.json -a 127.0.0.1:14553
   ```

This pipes the active telemetry connection stream coming from `/dev/ttyACM0` directly into the camera processing logic layer, populating the ground control station layout screens seamlessly.
