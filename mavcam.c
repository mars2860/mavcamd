#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mavcam.h"

// Protocol component configuration metrics
static uint8_t lib_system_id = 0;
static uint8_t lib_comp_id = MAV_COMP_ID_CAMERA;

// Internal runtime boot tick cache refreshed on every entry loop call
static uint32_t lib_current_boot_ms = 0;
// Internal runtime UTC cache refreshed on every entry loop call
static uint64_t lib_current_utc = 0;

// Core static MAVLink spec data banks
static mavlink_camera_information_t internal_cam_info;
static mavlink_camera_capture_status_t internal_cam_status;
static mavlink_camera_settings_t internal_cam_settings;
static mavlink_video_stream_information_t internal_streams[MAX_STREAMS];
static mavlink_storage_information_t internal_storages[MAX_STORAGES];
static uint8_t internal_stream_count = 0;
static uint8_t internal_storage_count = 0;
static uint8_t current_storage_id = 0;
static uint8_t current_stream_id = 0;

// Global tracking variable for elapsed video recording time
static uint32_t video_start_time_ms = 0;
// Global tracking variable for elapsed CAMERA_CAPTURE_STATUS time
uint32_t camera_capture_status_period_ms = DEFAULT_CAMERA_CAPTURE_PERIOD_MS;
uint32_t video_stream_status_period_ms = DEFAULT_VIDEO_STREAM_STATUS_PERIOD_MS;
// Count of taken images in current sequence
static uint32_t seq_image_count = 0;
// Target count of images in current sequence
static uint32_t seq_image_target_count = 0;
// Target interval of taking images in current sequence
static uint32_t seq_image_interval_ms = 0;
// Global tracking variable for captured image sequence counter
static uint32_t image_capture_sequence = 0;

// Telemetry state caching banks
static mavlink_global_position_int_t current_global_pos;
static mavlink_attitude_quaternion_t current_vehicle_attitude;
static mavlink_gimbal_device_attitude_status_t current_gimbal_status;

// Telemetry incoming update tracking tickers
static uint32_t last_gps_time_ms = 0;
static uint32_t last_attitude_time_ms = 0;
static uint32_t last_gimbal_time_ms = 0;

// Execution loop task schedule boundaries
static uint32_t last_heartbeat_time_ms = 0;
static uint32_t last_take_image_time_ms = 0;

static uint32_t last_capture_status_time_ms = 0;
static int current_stream_status_send_idx = 0;
static uint32_t last_stream_status_time_ms = 0;

// State validation suppressed flags
static bool telemetry_gps_valid = false;
static bool telemetry_attitude_valid = false;
static bool telemetry_gimbal_valid = false;

void mavcam_init(uint8_t system_id, uint8_t component_id) {
    lib_system_id = system_id;
    lib_comp_id = component_id;

    memset(&internal_cam_info, 0, sizeof(internal_cam_info));
    memset(&internal_cam_status, 0, sizeof(internal_cam_status));
    memset(&internal_cam_settings, 0, sizeof(internal_cam_settings));
    memset(internal_streams, 0, sizeof(internal_streams));
    memset(internal_storages, 0, sizeof(internal_storages));

    internal_cam_settings.mode_id = CAMERA_MODE_VIDEO;
    internal_cam_settings.focusLevel = NAN;
    internal_cam_settings.zoomLevel = NAN;
}

void mavcam_set_camera_information( const char *vendor_name,
                                    const char *model_name,
                                    uint32_t firmware_version,
                                    float focal_length,
                                    float sensor_size_h,
                                    float sensor_size_v,
                                    uint16_t resolution_h,
                                    uint16_t resolution_v,
                                    uint8_t lens_id,
                                    uint32_t flags,
                                    uint16_t cam_def_ver,
                                    const char *cam_def_uri,
                                    uint8_t gimbal_id,
                                    uint8_t camera_id) {

    (void)camera_id;    // It is not supported by this lib
    (void)gimbal_id;    // It is not supported by this lib

    if(vendor_name) {
        strncpy(    (char *)internal_cam_info.vendor_name,
                    vendor_name,
                    sizeof(internal_cam_info.vendor_name) - 1);
    }

    if(model_name) {
        strncpy(    (char *)internal_cam_info.model_name,
                    model_name,
                    sizeof(internal_cam_info.model_name) - 1);
    }

    if(cam_def_uri) {
        strncpy(    (char *)internal_cam_info.cam_definition_uri,
                    cam_def_uri,
                    sizeof(internal_cam_info.cam_definition_uri) - 1);   
    }

    internal_cam_info.firmware_version = firmware_version;
    internal_cam_info.focal_length = focal_length;
    internal_cam_info.sensor_size_h = sensor_size_h;
    internal_cam_info.sensor_size_v = sensor_size_v;
    internal_cam_info.resolution_h = resolution_h;
    internal_cam_info.resolution_v = resolution_v;
    internal_cam_info.lens_id = lens_id;
    internal_cam_info.flags = flags;
    internal_cam_info.cam_definition_version = cam_def_ver;
    internal_cam_info.gimbal_device_id = 0;
    internal_cam_info.camera_device_id = 0;
}

void mavcam_add_video_stream(   uint8_t stream_id,
                                uint8_t type,
                                uint16_t flags,
                                float framerate,
                                uint16_t resolution_h,
                                uint16_t resolution_v,
                                uint32_t bitrate,
                                uint16_t rotation,
                                uint16_t hfov,
                                uint8_t encoding,
                                const char *name,
                                const char *uri,
                                uint8_t camera_id) {
    (void)camera_id; // Is is not used in this library
    
    if (internal_stream_count >= MAX_STREAMS) {
        return;
    }

    int idx = internal_stream_count;

    internal_streams[idx].stream_id = stream_id;
    internal_streams[idx].type = type;
    internal_streams[idx].flags = flags;
    internal_streams[idx].framerate = framerate;
    internal_streams[idx].resolution_h = resolution_h;
    internal_streams[idx].resolution_v = resolution_v;
    internal_streams[idx].bitrate = bitrate;
    internal_streams[idx].rotation = rotation;
    internal_streams[idx].hfov = hfov;
    internal_streams[idx].encoding = encoding;
    internal_streams[idx].camera_device_id = camera_id;
    
    if(name) {
        strncpy(internal_streams[idx].name, name, sizeof(internal_streams[idx].name) - 1);
    }
    if(uri) {
        strncpy(internal_streams[idx].uri, uri, sizeof(internal_streams[idx].uri) - 1);
    }
    
    internal_stream_count++;
}

void mavcam_add_storage(    uint8_t storage_id,
                            uint8_t type,
                            float total_capacity,
                            const char *name,
                            uint32_t total_images,
                            float read_speed,
                            float write_speed,
                            uint8_t storage_usage) {
    if (internal_storage_count >= MAX_STORAGES) return;
    int idx = internal_storage_count;
    internal_storages[idx].storage_id = storage_id;
    internal_storages[idx].type = type;
    internal_storages[idx].storage_usage = storage_usage;
    internal_storages[idx].total_capacity = total_capacity;
    internal_storages[idx].read_speed = read_speed;
    internal_storages[idx].write_speed = write_speed;
    strncpy(internal_storages[idx].name, name, sizeof(internal_storages[idx].name) - 1);
    internal_storage_count++;

    if(idx == 0) {
        mavcam_set_current_storage(storage_id, total_images);
    }
}

void mavcam_set_current_storage(uint8_t storage_id, uint32_t total_images) {
    current_storage_id = storage_id;
    internal_cam_status.image_count = total_images;
    image_capture_sequence = total_images;
    LOG_INFO("Set current Storage ID %d, Total Images %d", storage_id, total_images);
}

// Low-level helper serializing and forwarding generated messages to the output callback
static void send_mavlink_msg(mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    mavcam_cb_send_raw_bytes(buf, len);
}

// Package and transmit immediate command transaction feedback acknowledgments
static void send_command_ack(uint16_t command, uint8_t result, uint8_t sender_sysid, uint8_t sender_compid) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
        lib_system_id,
        lib_comp_id,
        &msg,
        command,
        result,
        100,               // Progress in %
        0,                 // Additional result
        sender_sysid,      // System ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement.
        sender_compid      // Component ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement.
    );
    send_mavlink_msg(&msg);
}

// Package and transmit uniform component heartbeats
static void send_heartbeat(void) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        lib_system_id,
        lib_comp_id,
        &msg,
        MAV_TYPE_CAMERA,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        MAV_STATE_ACTIVE
    );
    send_mavlink_msg(&msg);
}

static void send_camera_information(void) {
    mavlink_message_t msg;

    internal_cam_info.time_boot_ms = lib_current_boot_ms;

    mavlink_msg_camera_information_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &internal_cam_info 
    );
    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent CAMERA_INFORMATION to router");
}

static int get_storage_idx(uint8_t storage_id) {
    int idx = -1;
    // Find the registered storage structure entry mapping the requested ID
    for (int i = 0; i < internal_storage_count; i++) {
        if (internal_storages[i].storage_id == storage_id) {
            idx = i;
            break;
        }
    }

    // Default fallback to the primary interface if specific tokens match 0 or broad sweeps
    if (idx < 0 && internal_storage_count > 0) {
        idx = 0;
    }

    return idx;
}

static int get_video_stream_idx(uint8_t stream_id) {
    int idx = -1;
    // Find the registered video stream structure entry mapping the requested ID
    for (int i = 0; i < internal_stream_count; i++) {
        if (internal_streams[i].stream_id == stream_id) {
            idx = i;
            break;
        }
    }

    // Default fallback to the primary interface if specific tokens match 0 or broad sweeps
    if (idx < 0 && internal_stream_count > 0) {
        idx = 0;
    }

    return idx;
}

static void send_storage_information(uint8_t storage_id) {
    int idx = get_storage_idx(storage_id);

    if (idx < 0) {
        LOG_ERROR("No storages configured to send status for ID: %d", storage_id);
        return;
    }

    mavlink_message_t msg;
    float used_mib = 0.f;
    float available_mib = 0.f;
    float total_mib = 0.f;

    if (mavcam_cb_query_storage_metrics(internal_storages[idx].storage_id, &used_mib, &available_mib, &total_mib)) {
        internal_storages[idx].used_capacity = used_mib;
        internal_storages[idx].available_capacity = available_mib;
        internal_storages[idx].status = STORAGE_STATUS_READY;
    } else {
        internal_storages[idx].status = STORAGE_STATUS_EMPTY;
    }

    internal_storages[idx].time_boot_ms = lib_current_boot_ms;
    internal_storages[idx].storage_count = internal_storage_count;

    mavlink_msg_storage_information_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &internal_storages[idx]);
    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent STORAGE_INFORMATION response for Storage ID: %d, Total MiB %.2f, Free MiB %.2f",
            internal_storages[idx].storage_id, 
            internal_storages[idx].total_capacity,
            internal_storages[idx].available_capacity);
}

static void send_camera_capture_status(void) {
    mavlink_message_t msg;
    float used_mib = 0.0;
    float available_mib = 0.f;
    float total_mib = 0.f;

    int storage_idx = get_storage_idx(current_storage_id);
    int stream_idx = get_video_stream_idx(current_stream_id);
    
    // Read dynamic metrics from the current storage device to report to standard telemetry packets
    if (internal_storage_count > 0 && storage_idx < internal_storage_count) {
        if(mavcam_cb_query_storage_metrics( internal_storages[storage_idx].storage_id,
                                            &used_mib,
                                            &available_mib,
                                            &total_mib) ) {
            internal_cam_status.available_capacity = available_mib;
        }
    }

    // Update video status
    if (internal_stream_count > 0 && stream_idx < internal_stream_count) {
        switch(mavcam_cb_video_query_status_is_recording(internal_streams[stream_idx].stream_id)) {
            case 0:
                internal_cam_status.video_status = VIDEO_CAPTURE_IDLE;
                break;
            case 1:
                internal_cam_status.video_status = VIDEO_CAPTURE_IN_PROGRESS;
                break;
        }
    }

    // Calculate active recording time dynamically if video capture is in progress
    if (internal_cam_status.video_status == VIDEO_CAPTURE_IN_PROGRESS) {
        internal_cam_status.recording_time_ms = lib_current_boot_ms - video_start_time_ms;
    }

    internal_cam_status.time_boot_ms = lib_current_boot_ms;
    internal_cam_status.image_interval = ((float)seq_image_interval_ms)/1000.f;

    mavlink_msg_camera_capture_status_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &internal_cam_status
    );

    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent CAMERA_CAPTURE_STATUS to router");
}

static void send_camera_settings(void) {
    mavlink_message_t msg;

    internal_cam_settings.time_boot_ms = lib_current_boot_ms;

    mavlink_msg_camera_settings_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &internal_cam_settings
    );
    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent CAMERA_SETTINGS to router (Mode: %d)", internal_cam_settings.mode_id);
}

static void send_video_stream_information(uint8_t stream_id) {
    int idx = -1;
    // Find the registered video stream structure entry mapping the requested ID
    for (int i = 0; i < internal_stream_count; i++) {
        if (internal_streams[i].stream_id == stream_id) {
            idx = i;
            break;
        }
    }

    // Default fallback to the primary interface if specific tokens match 0 or broad sweeps
    if (idx < 0 && internal_stream_count > 0) {
        idx = 0;
    }

    if (idx < 0) {
        LOG_ERROR("No video streams configured to send status for ID: %d", stream_id);
        return;
    }

    mavlink_message_t msg;

    internal_streams[idx].count = internal_stream_count;

    mavlink_msg_video_stream_information_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &internal_streams[idx]);
    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent VIDEO_STREAM_INFORMATION for Stream ID: %d", internal_streams[idx].stream_id);
}

static void send_video_stream_status(uint8_t stream_id) {
    int idx = -1;
    // Find the registered video stream structure entry mapping the requested ID
    for (int i = 0; i < internal_stream_count; i++) {
        if (internal_streams[i].stream_id == stream_id) {
            idx = i;
            break;
        }
    }

    // Default fallback to the primary interface if specific tokens match 0 or broad sweeps
    if (idx < 0 && internal_stream_count > 0) {
        idx = 0;
    }

    if (idx < 0) {
        LOG_ERROR("No video streams configured to send status for ID: %d", stream_id);
        return;
    }

    mavlink_message_t msg;
    mavlink_video_stream_status_t status;
    memset(&status, 0, sizeof(status));

    status.stream_id = internal_streams[idx].stream_id;
    status.flags = internal_streams[idx].flags;
    status.framerate = internal_streams[idx].framerate;
    status.resolution_h = internal_streams[idx].resolution_h;
    status.resolution_v = internal_streams[idx].resolution_v;
    status.bitrate = internal_streams[idx].bitrate;
    status.rotation = internal_streams[idx].rotation;
    status.hfov = internal_streams[idx].hfov;

    mavlink_msg_video_stream_status_encode(
        lib_system_id,
        lib_comp_id,
        &msg,
        &status);
    send_mavlink_msg(&msg);
    LOG_DEBUG("Sent VIDEO_STREAM_STATUS for Stream ID: %d", status.stream_id);
}

static void send_camera_image_captured(const camera_image_meta_t *meta) {
    mavlink_message_t msg;

    if(!meta) {
        LOG_ERROR("camera_image_meta_t is null");
        return;
    }

    mavlink_msg_camera_image_captured_pack(
        lib_system_id,
        lib_comp_id,
        &msg,
        meta->time_boot_ms,
        meta->time_utc,
        meta->camera_id,
        meta->lat,
        meta->lon,
        meta->alt,
        meta->relative_alt,
        meta->q,
        meta->image_index,
        MAV_BOOL_TRUE,
        meta->file_url
    );

    send_mavlink_msg(&msg);

    LOG_DEBUG("Sent CAMERA_IMAGE_CAPTURED idx: %d, boot: %d, lat: %d, lon: %d, amsl: %d, alt: %d, q: (%f,%f,%f,%f), file_url: %s", 
        meta->image_index,
        meta->time_boot_ms,
        meta->lat,
        meta->lon,
        meta->alt,
        meta->relative_alt,
        meta->q[0],
        meta->q[1],
        meta->q[2],
        meta->q[3],
        meta->file_url);
}

static void take_image(uint32_t sequence, uint8_t send_ack, uint8_t sender_sysid, uint8_t sender_compid) {
    camera_image_meta_t meta;
    memset(&meta, 0, sizeof(meta));

    last_take_image_time_ms = lib_current_boot_ms;

    internal_cam_status.image_status = IMAGE_CAPTURE_IN_PROGRESS;

    int32_t captured_image_idx = mavcam_cb_image_capture(sequence, meta.file_url, sizeof(meta.file_url));

    if (captured_image_idx > 0) {

        if(send_ack) {
            send_command_ack(MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
        }

        internal_cam_status.image_count++;
        seq_image_count++;

        meta.time_utc = lib_current_utc;
        meta.time_boot_ms = lib_current_boot_ms;
        meta.image_index = captured_image_idx;

        // Check GPS telemetry age and handle error log suppression
        if (lib_current_boot_ms - last_gps_time_ms > TELEMETRY_TIMEOUT_MS) {
            if (telemetry_gps_valid) {
                LOG_ERROR("GPS telemetry has just expired (Age: %u ms). Suppressing further timeout logs.",
                          lib_current_boot_ms - last_gps_time_ms);
                telemetry_gps_valid = false;
            }
            meta.lat = 0;
            meta.lon = 0;
            meta.alt = 0;
            meta.relative_alt = 0;
        } else {
            meta.lat = current_global_pos.lat;
            meta.lon = current_global_pos.lon;
            meta.alt = current_global_pos.alt;
            meta.relative_alt = current_global_pos.relative_alt;
        }

        // Check Gimbal telemetry age and handle advanced per-axis frame transformation logic
        uint32_t gimbal_age = lib_current_boot_ms - last_gimbal_time_ms;
        uint32_t attitude_age = lib_current_boot_ms - last_attitude_time_ms;

        if (gimbal_age > TELEMETRY_TIMEOUT_MS) {
            if (telemetry_gimbal_valid) {
                LOG_ERROR("Gimbal telemetry expired (Age: %u ms). Resetting to identity quaternion.", gimbal_age);
                telemetry_gimbal_valid = false;
            }
            meta.q[0] = 1.0f; meta.q[1] = 0.0f; meta.q[2] = 0.0f; meta.q[3] = 0.0f;
        } else {
            float gimbal_roll = 0.0f, gimbal_pitch = 0.0f, gimbal_yaw = 0.0f;
            
            // Unpack gimbal quaternion into Euler angles
            mavlink_quaternion_to_euler(current_gimbal_status.q, &gimbal_roll, &gimbal_pitch, &gimbal_yaw);

            // Variables to hold the computed absolute world-relative orientations
            float final_world_roll  = gimbal_roll;
            float final_world_pitch = gimbal_pitch;
            float final_world_yaw   = gimbal_yaw;

            // Check if any axis is relative to the vehicle body frame instead of earth horizon
            bool needs_vehicle_baseline = !(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_ROLL_LOCK)  ||
                                          !(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_PITCH_LOCK) ||
                                          !(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK);

            if (needs_vehicle_baseline) {
                if (attitude_age > TELEMETRY_TIMEOUT_MS) {
                    if (telemetry_attitude_valid) {
                        LOG_ERROR("Vehicle attitude expired (Age: %u ms) during mixed-frame conversion.", attitude_age);
                        telemetry_attitude_valid = false;
                    }
                    // If vehicle baseline is missing, fallback directly to raw uncompensated gimbal inputs
                } else {
                    // Map the official MAVLink message fields onto the standard array layout
                    // Note: MAVLink packs elements as q1 (w), q2 (x), q3 (y), q4 (z)
                    float vehicle_q[4] = {
                        current_vehicle_attitude.q1,
                        current_vehicle_attitude.q2,
                        current_vehicle_attitude.q3,
                        current_vehicle_attitude.q4
                    };
                    float vehicle_roll = 0.0f, vehicle_pitch = 0.0f, vehicle_yaw = 0.0f;
                    
                    // Unpack vehicle attitude quaternion
                    mavlink_quaternion_to_euler(vehicle_q, &vehicle_roll, &vehicle_pitch, &vehicle_yaw);

                    // Perform independent per-axis accumulation depending on MAVLink lock status flags
                    if (!(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_ROLL_LOCK)) {
                        final_world_roll = vehicle_roll + gimbal_roll;
                    }
                    if (!(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_PITCH_LOCK)) {
                        final_world_pitch = vehicle_pitch + gimbal_pitch;
                    }
                    if (!(current_gimbal_status.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK)) {
                        final_world_yaw = vehicle_yaw + gimbal_yaw;
                    }
                }
            }

            // Repack absolute angles into the metadata array
            mavlink_euler_to_quaternion(final_world_roll, final_world_pitch, final_world_yaw, meta.q);
        }

        // Persist the collected meta block directly onto the filesystem disk tracker
        mavcam_cb_save_image_metadata(&meta);
        // Instantly notify flight controllers that the file operation concluded
        send_camera_image_captured(&meta);
    } else {
        if(send_ack) {
            send_command_ack(MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_FAILED, sender_sysid, sender_compid);
        }
        LOG_ERROR("Internal engine failed to capture photo frame artifact via storage block requests");
    }

    if(seq_image_count >= seq_image_target_count && seq_image_target_count > 0) {
        internal_cam_status.image_status = IMAGE_CAPTURE_IDLE;
        seq_image_target_count = 0;
        seq_image_interval_ms = 0;
    }
}

static void on_cmd_image_start_capture(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t camera_id = (uint8_t)cmd->param1;
    uint32_t interval_ms = (uint32_t)(cmd->param2*1000.f);
    uint32_t count = (uint32_t)cmd->param3;
    uint32_t sequence = (uint32_t)cmd->param4;
    
    LOG_DEBUG("Received MAV_CMD_IMAGE_START_CAPTURE (Cam ID: %d, Int: %d, Count: %d, Seq: %d)",
                            camera_id, interval_ms, count, sequence);
    
    seq_image_interval_ms = interval_ms;
    seq_image_target_count = count;
    seq_image_count = 0;

    if(count > 1) sequence = 0;

    take_image(sequence, true, sender_sysid, sender_compid);
}

static void on_cmd_image_stop_capture(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t camera_id = (uint8_t)cmd->param1;
    
    LOG_DEBUG("Received MAV_CMD_IMAGE_STOP_CAPTURE (Cam ID: %d)",camera_id);
    
    send_command_ack(MAV_CMD_IMAGE_STOP_CAPTURE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);

    seq_image_interval_ms = 0;
    seq_image_target_count = 0;
    seq_image_count = 0;

    internal_cam_status.image_status = IMAGE_CAPTURE_IDLE;
}

static void on_cmd_set_camera_mode(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t camera_id = (uint8_t)cmd->param1;
    uint8_t mode = (uint8_t)cmd->param2;

    LOG_DEBUG("Received MAV_CMD_SET_CAMERA_MODE (Mode: %d, Cam ID: %d)", mode, camera_id);

    if (mavcam_cb_set_camera_mode(mode)) {
        internal_cam_settings.mode_id = mode;
        send_command_ack(MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
        LOG_DEBUG("Camera mode is updated successfully");
        send_camera_settings();
    } else {
        send_command_ack(MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_FAILED, sender_sysid, sender_compid);
        LOG_ERROR("Failed to update camera mode");
    }
}

static void on_cmd_video_start_capture(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t stream_id = (uint8_t)cmd->param1;
    uint8_t status_freq = (uint8_t)cmd->param2;
    uint8_t camera_id = (uint8_t)cmd->param3;
    
    LOG_DEBUG("Received MAV_CMD_VIDEO_START_CAPTURE (Stream ID: %d, Status Freq: %d, Camera ID: %d)",
                            stream_id, status_freq, camera_id);

    for(uint8_t i = 0; i < internal_stream_count; i++) {
        if(stream_id == 0 || stream_id == internal_streams[i].stream_id) {
            if (mavcam_cb_video_start_capture(i)) {
                send_command_ack(MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
                // In the MAVLink Camera Protocol v2, I don't see a flag indicating video_capture_in_progress for an individual stream_id.
                // I see only one flag for all streams. Maybe this is a shortcoming in the MAVLink Camera Protocol v2?
                internal_cam_status.video_status = VIDEO_CAPTURE_IN_PROGRESS;
                video_start_time_ms = lib_current_boot_ms; 
                last_capture_status_time_ms = lib_current_boot_ms;
                camera_capture_status_period_ms = DEFAULT_CAMERA_CAPTURE_PERIOD_MS;
                if(status_freq > 0) {
                    camera_capture_status_period_ms = 1000 / status_freq;
                }
                send_camera_capture_status();
            } else {
                send_command_ack(MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_FAILED, sender_sysid, sender_compid);
                LOG_ERROR("Failed to start video capture. Rejecting MAVLink command.");
            }
        }
    }
}

static void on_cmd_video_stop_capture(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t stream_id = (uint8_t)cmd->param1;
    uint8_t camera_id = (uint8_t)cmd->param2;
    
    LOG_DEBUG("Received MAV_CMD_VIDEO_STOP_CAPTURE (Stream ID: %d, Camera ID: %d)", stream_id, camera_id);

    for(uint8_t i = 0; i < internal_stream_count; i++) {
        if(stream_id == 0 || stream_id == internal_streams[i].stream_id) {
            if (mavcam_cb_video_stop_capture(i)) {
                send_command_ack(MAV_CMD_VIDEO_STOP_CAPTURE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
                internal_cam_status.video_status = VIDEO_CAPTURE_IDLE;
                video_start_time_ms = 0;
                send_camera_capture_status();
            } else {
                send_command_ack(MAV_CMD_VIDEO_STOP_CAPTURE, MAV_RESULT_FAILED, sender_sysid, sender_compid);
                LOG_ERROR("Failed to stop video capture. Rejecting MAVLink command.");
            }
        }
    }
}

static void on_cmd_video_start_stream(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t stream_id = (uint8_t)cmd->param1;
    uint8_t camera_id = (uint8_t)cmd->param2;
    
    LOG_DEBUG("Received MAV_CMD_VIDEO_START_STREAMING (Stream ID: %d, Cam ID: %d)", stream_id, camera_id);
    
    send_command_ack(MAV_CMD_VIDEO_START_STREAMING, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
    
    for (int i = 0; i < internal_stream_count; i++) {
        if (stream_id == 0 || internal_streams[i].stream_id == stream_id) {
            if(mavcam_cb_video_start_stream(internal_streams[i].stream_id)) {
                internal_streams[i].flags |= VIDEO_STREAM_STATUS_FLAGS_RUNNING;
            }
            send_video_stream_status(internal_streams[i].stream_id);
        }
    }
}

static void on_cmd_video_stop_stream(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t stream_id = (uint8_t)cmd->param1;
    uint8_t camera_id = (uint8_t)cmd->param2;
    
    LOG_DEBUG("Received MAV_CMD_VIDEO_STOP_STREAMING (Stream ID: %d, Cam ID: %d)", stream_id, camera_id);
    
    send_command_ack(MAV_CMD_VIDEO_STOP_STREAMING, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
    
    for (int i = 0; i < internal_stream_count; i++) {
        if (stream_id == 0 || internal_streams[i].stream_id == stream_id) {
            if( mavcam_cb_video_stop_stream(internal_streams[i].stream_id) && 
                (internal_streams[i].flags & VIDEO_STREAM_STATUS_FLAGS_RUNNING) ) {
                internal_streams[i].flags ^= VIDEO_STREAM_STATUS_FLAGS_RUNNING;
            }
            send_video_stream_status(internal_streams[i].stream_id);
        }
    }
}

static void on_cmd_reset_camera_settings(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t reset_mode = (uint8_t)cmd->param1;
    LOG_DEBUG("Received MAV_CMD_RESET_CAMERA_SETTINGS (Reset Mode: %d)", reset_mode);

    if (mavcam_cb_reset_camera_settings(reset_mode)) {
        send_command_ack(MAV_CMD_RESET_CAMERA_SETTINGS, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
        internal_cam_settings.mode_id = CAMERA_MODE_VIDEO; 
        send_camera_settings();
    } else {
        send_command_ack(MAV_CMD_RESET_CAMERA_SETTINGS, MAV_RESULT_FAILED, sender_sysid, sender_compid);
    }
}

static void on_cmd_storage_format(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint8_t storage_id = (uint8_t)cmd->param1;
    uint8_t format = (uint8_t)cmd->param2;
    uint8_t reset = (uint8_t)cmd->param3;

    LOG_DEBUG("Received MAV_CMD_STORAGE_FORMAT (Storage ID: %d, Format: %d, Reset: %d)", storage_id, format, reset);

    if(format) {
        for(uint8_t idx = 0; idx < internal_storage_count; idx++) {
            if(storage_id == 0 || internal_storages[idx].storage_id == storage_id) {
                if (mavcam_cb_storage_format(storage_id)) {
                    send_command_ack(MAV_CMD_STORAGE_FORMAT, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
        
                    internal_cam_status.image_count = 0;
                    image_capture_sequence = 0;

                    send_camera_capture_status();
                    send_storage_information(internal_storages[idx].storage_id);
                } else {
                    send_command_ack(MAV_CMD_STORAGE_FORMAT, MAV_RESULT_FAILED, sender_sysid, sender_compid);
                }
            }
        }
    }

    if(reset) {
        send_command_ack(MAV_CMD_STORAGE_FORMAT, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);

        internal_cam_status.image_count = 0;
        image_capture_sequence = 0;

        send_camera_capture_status();
        
        for(uint8_t idx = 0; idx < internal_storage_count; idx++) {
            if(storage_id == 0 || internal_storages[idx].storage_id == storage_id) {
                send_storage_information(internal_storages[idx].storage_id);
            }
        }
    }
}

static void on_cmd_set_message_interval(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    int msg_id = cmd->param1;
    int interval_us = cmd->param2;
    LOG_DEBUG("Received MAV_CMD_SET_MESSAGE_INTERVAL (MsgId: %d, Interval: %d us, p3=%.1f,p4=%1.f,p5=%.1f,p6=%.1f,p7=%.1f)",
            msg_id,interval_us,cmd->param3,cmd->param4,cmd->param5,cmd->param6,cmd->param7);
    
    switch(msg_id) {
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            
            if(interval_us < 0) // -1 means stop message streaming
                camera_capture_status_period_ms = 0;
            else if(interval_us == 0)   // 0 means default rate
                camera_capture_status_period_ms = DEFAULT_CAMERA_CAPTURE_PERIOD_MS;
            else
                camera_capture_status_period_ms = interval_us/1000;
            
            LOG_DEBUG("Set send period=%d ms for CAMERA_CAPTURE_STATUS message",camera_capture_status_period_ms);
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
            break;
        case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:

            if(interval_us < 0) // -1 means stop message streaming
                video_stream_status_period_ms = 0;
            else if(interval_us == 0)   // 0 means default rate
                video_stream_status_period_ms = DEFAULT_VIDEO_STREAM_STATUS_PERIOD_MS;
            else
                video_stream_status_period_ms = interval_us/1000;

            LOG_DEBUG("Set send period=%d ms for VIDEO_STREAM_STATUS message",video_stream_status_period_ms);
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
            break;
        default:
            LOG_ERROR("Requested message stream is not supported");
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
            break;
    }
}

static void on_req_msg_camera_information(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    (void)cmd;
    LOG_DEBUG("Received request for CAMERA_INFORMATION");
    send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
    send_camera_information();
}

static void on_req_msg_camera_capture_status(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    (void)cmd;
    LOG_DEBUG("Received request for CAMERA_CAPTURE_STATUS");
    send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
    send_camera_capture_status();
}

static void on_req_msg_camera_capture_settings(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    (void)cmd;
    LOG_DEBUG("Received request for CAMERA_SETTINGS");
    send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
    send_camera_settings();
}

static void on_req_msg_storage_information(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    bool success = false;
    uint8_t storage_id = (uint8_t)cmd->param2;
    LOG_DEBUG("Received request for STORAGE_INFORMATION (Storage ID: %d)", storage_id);
    for (uint8_t i = 0; i < internal_storage_count; i++) {
        if (storage_id == 0 || internal_storages[i].storage_id == storage_id) {
            if(!success) {
                send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
                success = true;
            }
            send_storage_information(internal_storages[i].storage_id);
        }
    }
    if(!success) {
        LOG_ERROR("Requested STORAGE_INFORMATION (Storage ID: %d) is not found", storage_id);
        send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
    }
}

static void on_req_msg_video_stream_information(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    bool success = false;
    uint8_t stream_id = (uint8_t)cmd->param2;
    LOG_DEBUG("Received request for VIDEO_STREAM_INFORMATION (Stream ID: %d)", stream_id);
    for (uint8_t i = 0; i < internal_stream_count; i++) {
        if (stream_id == 0 || internal_streams[i].stream_id == stream_id) {
            if(!success) {
                send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
                success = true;
            }
            send_video_stream_information(internal_streams[i].stream_id);
        }
    }
    if(!success) {
        LOG_ERROR("Requested VIDEO_STREAM_INFORMATION (Stream ID: %d) is not found", stream_id);
        send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
    }
}

static void on_req_msg_video_stream_status(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    bool success = false;
    uint8_t stream_id = (uint8_t)cmd->param2;
    LOG_DEBUG("Received request for VIDEO_STREAM_STATUS (Stream ID: %d)", stream_id);
    for (uint8_t i = 0; i < internal_stream_count; i++) {
        if (stream_id == 0 || internal_streams[i].stream_id == stream_id) {
            if(!success) {
                send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
                success = true;
            }
            send_video_stream_status(internal_streams[i].stream_id);
        }
    }
    if(!success) {
        LOG_ERROR("Requested VIDEO_STREAM_STATUS (Stream ID: %d) is not found", stream_id);
        send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
    }
}

static void on_req_msg_camera_image_captured(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    int32_t target_image_sequence = (int32_t)cmd->param2;
    int32_t range = (int32_t)cmd->param3;
    LOG_DEBUG("Received request for CAMERA_IMAGE_CAPTURED (Seq: %d, Rng: %d)", target_image_sequence, range);
                                        
    if (target_image_sequence < 0) {
        LOG_ERROR("Requesting all CAMERA_IMAGE_CAPTURED messages is not supported");
        send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
    } else if (range != 0) {
        LOG_ERROR("Requesting a range of CAMERA_IMAGE_CAPTURED messages is not supported");
        send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
    } else {
        camera_image_meta_t cached_meta;
        // Attempt to recover historical metadata payload directly from the storage disk
        if (mavcam_cb_load_image_metadata_by_index(target_image_sequence, &cached_meta)) {
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, sender_sysid, sender_compid);
            send_camera_image_captured(&cached_meta);
        } else {
            // Requested image meta block could not be located or parsed from storage files
            LOG_ERROR("Metadata file for requested index %d is not found", target_image_sequence);
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, sender_sysid, sender_compid);
        }
    }    
}

static void process_request_cmd(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    uint32_t req_msg_id = (uint32_t)cmd->param1;
    switch(req_msg_id) {
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            on_req_msg_camera_information(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            on_req_msg_camera_capture_status(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_CAMERA_SETTINGS:
            on_req_msg_camera_capture_settings(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_STORAGE_INFORMATION:
            on_req_msg_storage_information(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
            on_req_msg_video_stream_information(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:
            on_req_msg_video_stream_status(sender_sysid, sender_compid, cmd);
            break;
        case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
            on_req_msg_camera_image_captured(sender_sysid, sender_compid, cmd);
            break;
        default:
            //LOG_DEBUG("Received unsupported cmd=%d,p1=%.1f,p2=%.1f,p3=%.1f,p4=%.1f,p5=%.1f,p6=%.1f,p7=%.1f",
            //    cmd->command,cmd->param1,cmd->param2,cmd->param3,cmd->param4,cmd->param5,cmd->param6,cmd->param7);
            send_command_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED, sender_sysid, sender_compid);
            break;
    }
}

// Unmarshal execution maps for parsed tracking block data arrays
static void process_command_long(uint8_t sender_sysid, uint8_t sender_compid, mavlink_command_long_t *cmd) {
    if (cmd->target_component != lib_comp_id && 
        cmd->target_component != MAV_COMP_ID_ALL) {
        return;
    }

    switch (cmd->command) {
        case MAV_CMD_SET_CAMERA_MODE:
            on_cmd_set_camera_mode(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_VIDEO_START_CAPTURE:
            on_cmd_video_start_capture(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_VIDEO_STOP_CAPTURE:
            on_cmd_video_stop_capture(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_IMAGE_START_CAPTURE:
            on_cmd_image_start_capture(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_IMAGE_STOP_CAPTURE:
            on_cmd_image_stop_capture(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_VIDEO_START_STREAMING:
            on_cmd_video_start_stream(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_VIDEO_STOP_STREAMING:
            on_cmd_video_stop_stream(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_STORAGE_FORMAT:
            on_cmd_storage_format(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_RESET_CAMERA_SETTINGS:
            on_cmd_reset_camera_settings(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_SET_MESSAGE_INTERVAL:
            on_cmd_set_message_interval(sender_sysid, sender_compid, cmd);
            break;
        case MAV_CMD_REQUEST_MESSAGE:
            process_request_cmd(sender_sysid, sender_compid, cmd);
            break;
    default:
        //LOG_DEBUG("Received unsupported cmd=%d,p1=%.1f,p2=%.1f,p3=%.1f,p4=%.1f,p5=%.1f,p6=%.1f,p7=%.1f",
        //    cmd->command,cmd->param1,cmd->param2,cmd->param3,cmd->param4,cmd->param5,cmd->param6,cmd->param7);
        send_command_ack(cmd->command, MAV_RESULT_UNSUPPORTED, sender_sysid, sender_compid);
        break;
    }
}

static void on_msg_hearbeat(mavlink_message_t *rx_msg) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(rx_msg, &hb);
    if (hb.autopilot != MAV_AUTOPILOT_INVALID && !lib_system_id) {
        lib_system_id = rx_msg->sysid;
        LOG_INFO("Learned System ID from Autopilot Heartbeat: %d (CompID: %d, Type: %d)",
                    lib_system_id, rx_msg->compid, hb.type);
    }
}

static void on_msg_global_position(mavlink_message_t *rx_msg) {
    mavlink_msg_global_position_int_decode(rx_msg, &current_global_pos);
    last_gps_time_ms = lib_current_boot_ms;
    if (!telemetry_gps_valid) {
        LOG_INFO("GPS telemetry connection established/restored");
        telemetry_gps_valid = true;
    }
}

static void on_msg_attitude_quaternion(mavlink_message_t *rx_msg) {
    mavlink_msg_attitude_quaternion_decode(rx_msg, &current_vehicle_attitude);
    last_attitude_time_ms = lib_current_boot_ms;
    if (!telemetry_attitude_valid) {
        LOG_INFO("Vehicle attitude quaternion connection established/restored");
        telemetry_attitude_valid = true;
    }
}

static void on_msg_gimbal_device_attitude_status(mavlink_message_t *rx_msg) {
    mavlink_gimbal_device_attitude_status_t gimbal;
    mavlink_msg_gimbal_device_attitude_status_decode(rx_msg, &gimbal);

    // this code has been commented because
    // internal_cam_info.gimbal_device_id requires a connected gimbal,
    // forcing this camera component to act as a gimbal manager
    //if(gimbal.gimbal_device_id != internal_cam_info.gimbal_device_id)
    //    return;
    
    if(gimbal.gimbal_device_id != CAMERA_GIMBAL_DEVICE_ID)
        return;

    current_gimbal_status = gimbal;

    last_gimbal_time_ms = lib_current_boot_ms;
    if (!telemetry_gimbal_valid) {
        LOG_INFO("Gimbal telemetry connection established/restored");
        telemetry_gimbal_valid = true;
    }
}

static void on_msg_system_time(mavlink_message_t *rx_msg) {
    mavlink_system_time_t sys_time;
    mavlink_msg_system_time_decode(rx_msg, &sys_time);
    
    // Pass the extracted UTC time in microseconds to the hardware layer callback
    //LOG_DEBUG("Received SYSTEM_TIME from sysid %d. Unix time us: %lui", rx_msg->sysid, sys_time.time_unix_usec);
    
    if (sys_time.time_unix_usec > 0) {
        mavcam_cb_set_system_time(sys_time.time_unix_usec);
    }
}

void mavcam_update(const uint8_t *data, size_t data_size, uint32_t boot_ms, uint64_t utc_us) {
    mavlink_status_t status;
    mavlink_message_t rx_msg;
    lib_current_boot_ms = boot_ms;
    lib_current_utc = utc_us;
    for (size_t i = 0; i < data_size; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &rx_msg, &status)) {

            switch(rx_msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    on_msg_hearbeat(&rx_msg);
                    break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&rx_msg, &cmd);
                    process_command_long(rx_msg.sysid, rx_msg.compid, &cmd);
                    break;
                case MAVLINK_MSG_ID_SYSTEM_TIME:
                    on_msg_system_time(&rx_msg);
                    break;
            }

            if(rx_msg.sysid == lib_system_id) {
                switch(rx_msg.msgid) {
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        on_msg_global_position(&rx_msg);
                        break;
                    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                        on_msg_attitude_quaternion(&rx_msg);
                        break;
                    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
                        on_msg_gimbal_device_attitude_status(&rx_msg);
                        break;
                }
            }
        }
    }
}

void mavcam_spin(uint32_t boot_ms, uint64_t utc_us) {
    lib_current_boot_ms = boot_ms;
    lib_current_utc = utc_us;

    if (lib_current_boot_ms - last_heartbeat_time_ms >= DEFAULT_HEARTBEAT_PERIOD_MS) {
        send_heartbeat();
        last_heartbeat_time_ms = lib_current_boot_ms;
    }

    if (    lib_system_id > 0 &&
            internal_stream_count > 0 && 
            video_stream_status_period_ms > 0 &&
            current_stream_status_send_idx < internal_stream_count &&
            (lib_current_boot_ms - last_stream_status_time_ms >= video_stream_status_period_ms)) {
                
        send_video_stream_status(internal_streams[current_stream_status_send_idx].stream_id);
        current_stream_status_send_idx++;
        if (current_stream_status_send_idx >= internal_stream_count) {
            current_stream_status_send_idx = 0;
        }
        last_stream_status_time_ms = lib_current_boot_ms;
    }

    if ( lib_system_id > 0 &&
         camera_capture_status_period_ms > 0 &&
         lib_current_boot_ms - last_capture_status_time_ms >= camera_capture_status_period_ms) {
            
        send_camera_capture_status();
        last_capture_status_time_ms = lib_current_boot_ms;
    }

    if( seq_image_interval_ms > 0 &&
        (seq_image_count < seq_image_target_count || seq_image_target_count == 0) &&
        (lib_current_boot_ms - last_take_image_time_ms) >= seq_image_interval_ms) {
        take_image(0,false,0,0);
    }
}

mavlink_camera_information_t mavcam_get_camera_information(void) {
    return internal_cam_info;
}

mavlink_video_stream_information_t mavcam_get_video_stream_information(uint8_t stream_id) {
    int idx = get_video_stream_idx(stream_id);

    if(idx >= internal_stream_count || idx < 0)
        idx = 0;

    return internal_streams[idx];
}

mavlink_storage_information_t mavcam_get_storage_information(uint8_t storage_id) {
    int idx = get_video_stream_idx(storage_id);

    if(idx >= internal_storage_count || idx < 0)
        idx = 0;

    return internal_storages[idx];
}

uint8_t mavcam_get_video_stream_count(void) {
    return internal_stream_count;
}

uint8_t mavcam_get_storage_count(void) {
    return internal_storage_count;
}

uint8_t mavcam_get_sysid(void) {
    return lib_system_id;
}

uint8_t mavcam_get_compid(void) {
    return lib_comp_id;
}