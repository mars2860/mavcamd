/**
 * @file mavcam.h
 * @brief MAVLink Camera Protocol v2 Component Library.
 * @details This library decouples the MAVLink protocol state machine from 
 * underlying transport layers, storage systems, and hardware-specific REST APIs.
 */

#ifndef MAVCAM_H
#define MAVCAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <common/mavlink.h>
#include <mavlink_conversions.h>
#include "macrologger.h"

/// Maximum supported number of concurrent video streams
#define MAX_STREAMS 2

/// Maximum supported number of concurrent storage devices (eMMC, SD card, etc.).
#define MAX_STORAGES 2

/// Interval for sending periodic MAVLink HEARTBEAT messages in milliseconds.
#define DEFAULT_HEARTBEAT_PERIOD_MS       2000

/// Interval for sending periodic MAVLink VIDEO_STREAM_STATUS messages in milliseconds (0 to send on request only).
#define DEFAULT_VIDEO_STREAM_STATUS_PERIOD_MS    0

/// Default interval for periodic CAMERA_CAPTURE_STATUS messages in milliseconds (0 to send on request only).
#define DEFAULT_CAMERA_CAPTURE_PERIOD_MS    2000

/// Maximum permitted age of vehicle/gimbal telemetry data in milliseconds before it is considered stale.
#define TELEMETRY_TIMEOUT_MS                3000

#define CAMERA_GIMBAL_DEVICE_ID             1

/**
 * @enum video_status_t
 * @brief Represents the current video recording state of a stream.
 */
typedef enum {
    VIDEO_CAPTURE_IDLE = 0,         ///< Video recording is stopped.
    VIDEO_CAPTURE_IN_PROGRESS = 1   ///< Video recording is actively running.
} video_status_t;

/**
 * @enum image_status_t
 * @brief Represents the current image capture state of the camera.
 */
typedef enum {
    IMAGE_CAPTURE_IDLE = 0,                 ///< Image capture is idle.
    IMAGE_CAPTURE_IN_PROGRESS = 1,          ///< Single image capture is in progress.
    IMAGE_CAPTURE_INTERVAL_IDLE = 2,        ///< Interval mode is set but currently idle.
    IMAGE_CAPTURE_INTERVAL_IN_PROGRESS = 3  ///< Interval capture is actively running.
} image_status_t;

/**
 * @struct camera_image_meta_t
 * @brief Structure defining captured image metadata persisted onto block storage side-by-side with images.
 */
typedef struct __attribute__((packed)) {
    uint64_t time_utc;           ///< Timestamp (time since UNIX epoch) in microseconds.
    uint32_t time_boot_ms;       ///< Timestamp (time since system boot) in milliseconds.
    int32_t lat;                 ///< Latitude where image was taken in degrees * 1E7 (0 if not available).
    int32_t lon;                 ///< Longitude where image was taken in degrees * 1E7 (0 if not available).
    int32_t alt;                 ///< Altitude (MSL) where image was taken in mm (0 if not available).
    int32_t relative_alt;        ///< Altitude above ground level in mm (0 if not available).
    float q[4];                  ///< Quaternion orientation of the camera (w, x, y, z) in Earth-fixed frame.
    int32_t image_index;         ///< Image index sequence since capture started (strictly int32_t per standard).
    uint8_t camera_id;           ///< Camera ID component matching specific sensor hardware.
    char file_url[205];          ///< Fixed-size array buffer storing the absolute local file path or remote URL.
} camera_image_meta_t;


/* ========================================================================== */
/*           SECTION 1: USER-DEFINED HARDWARE CALLBACK INTERFACE             */
/* ========================================================================== */

/**
 * @brief Request the hardware layer to begin video recording capture on a specific stream.
 * @param[in] stream_id Target video stream identifier parsed from configuration.
 * @return true if the command was accepted by the hardware layer, false otherwise.
 */
bool mavcam_cb_video_start_capture(uint8_t stream_id);

/**
 * @brief Request the hardware layer to terminate active video recording capture.
 * @param[in] stream_id Target video stream identifier.
 * @return true if the command was accepted by the hardware layer, false otherwise.
 */
bool mavcam_cb_video_stop_capture(uint8_t stream_id);

/**
 * @brief Request the hardware/platform layer to activate network video streaming for a specific channel.
 * @details This function is triggered when a ground control station or companion computer requests 
 *          immediate activation of raw or encoded video broadcasting (e.g., spinning up an RTSP, RTP, 
 *          UDP stream channel server pipeline).
 * @param[in] stream_id Unique active network channel identifier profile to be started.
 * @return true if the streaming engine successfully initialized and launched the stream, false otherwise.
 */
bool mavcam_cb_video_start_stream(uint8_t stream_id);

/**
 * @brief Request the hardware/platform layer to terminate network video streaming for a specific channel.
 * @details This function is triggered when the ground control station turns off the video feed window 
 *          or requests bandwidth conservation, allowing the platform to shutdown encoder pipelines or 
 *          network sockets associated with the transmission channel.
 * @param[in] stream_id Unique active network channel identifier profile to be stopped.
 * @return true if the stream channel was successfully stopped or de-initialized, false otherwise.
 */
bool mavcam_cb_video_stop_stream(uint8_t stream_id);

/**
 * @brief Query the hardware layer to verify if video recording is actively running.
 * @param[in] stream_id Target video stream identifier to check.
 * @return -1 means the method execution is failed, 0 means recording is not active, 1 means recording is active
 */
int32_t mavcam_cb_video_query_status_is_recording(uint8_t stream_id);

/**
 * @brief Request the hardware layer to execute an image capture command block sequence.
 * @param[in] sequence Capture sequence number starting from 1 or 0 to auto-increment
 * @param[inout] file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface. It has to be char[205]
 * @param[in] file_url_size The size of file_url
 * @return Captured image index or -1 if it has been failed
 */
int32_t mavcam_cb_image_capture(uint32_t sequence, char *file_url, size_t file_url_size);

/**
 * @brief Request the hardware layer to transition into a specific operational camera mode.
 * @param[in] camera_mode Target camera operational mode (0: Photo, 1: Video, 2: Survey).
 * @return true if the mode transition was successfully completed, false otherwise.
 */
bool mavcam_cb_set_camera_mode(uint8_t camera_mode);

/**
 * @brief Query the hardware layer to fetch real-time free memory metrics for a storage partition.
 * @details The library uses this callback to update dynamically changing available/used capacities.
 * @param[in] storage_id Target storage slot or backend block device identifier.
 * @param[out] used_mib Pointer to store the computed used allocation capacity scaled in Mebibytes (MiB).
 * @param[out] available_mib Pointer to store the computed remaining free capacity scaled in Mebibytes (MiB).
 * @param[out] total_mib Pointer to store the total capacity scaled in Mebibytes (MiB).
 * @return true if storage metrics were successfully queried, false otherwise.
 */
bool mavcam_cb_query_storage_metrics(uint8_t storage_id, float *used_mib, float *available_mib, float *total_mib);

/**
 * @brief Low-level network interface output wrapper utilized by the library to transmit serialized data.
 * @details This callback must be implemented by user code to pipe raw MAVLink packet streams into UDP sockets or UART interfaces.
 * @param[in] buffer Pointer to the continuous byte array containing the serialized packet stream.
 * @param[in] length Total size of the payload buffer block package in bytes.
 */
void mavcam_cb_send_raw_bytes(const uint8_t *buffer, uint16_t length);

/**
 * @brief Callback interface to persist captured image metadata side-by-side with the image file.
 * @details This function is invoked by the library immediately after a successful photo capture
 *          to serialize and dump the collected telemetry matrix (GPS, orientation, timestamps) onto disk.
 *          The implementation should typically strip the image extension and append a custom `.meta` suffix.
 * @param[in] meta Pointer to the structured binary block containing the fully populated photo telemetry metadata.
 * @return true if the metadata block was completely and successfully written to disk, false otherwise.
 */
bool mavcam_cb_save_image_metadata(const camera_image_meta_t *meta);

/**
 * @brief Callback interface to look up and read photo metadata from disk using a unique sequence index.
 * @details This function is invoked by the library when a ground control station or an external component 
 *          requests telemetry log verification for a specific captured photo frame. The platform layer 
 *          must scan the active storage directory, locate the corresponding metadata binary file (.meta) 
 *          matching the requested sequence index, and unpack its content into the output buffer.
 * @param[in] index Unique signed sequential frame index to search for on the block storage filesystem.
 * @param[out] meta_out Pointer to the target structure to be populated with retrieved telemetry metadata parameters.
 * @return true if the matching metadata file was successfully found, opened, and parsed; false otherwise.
 */
bool mavcam_cb_load_image_metadata_by_index(int32_t index, camera_image_meta_t *meta_out);

/**
 * @brief Request the hardware layer to reset all camera settings to factory defaults.
 * @param[in] reset_mode Reset all settings if set to 1
 * @return true if the command was accepted by the hardware layer, false otherwise.
 */
bool mavcam_cb_reset_camera_settings(uint8_t reset_mode);

/**
 * @brief Request the hardware layer to format a specific storage device.
 * @param[in] storage_id Target storage slot identifier.
 * @return true if formatting succeeded, false otherwise.
 */
bool mavcam_cb_storage_format(uint8_t storage_id);

/* ========================================================================== */
/*                 SECTION 2: CORE LIBRARY API PROTOTYPES                     */
/* ========================================================================== */

/**
 * @brief Initialize the core MAVLink camera protocol state machine engine weights and parameters.
 * @param[in] system_id Initial system ID for the component (can be 0 to dynamically learn from heartbeat of autopilot).
 * @param[in] component_id Specific MAVLink component ID to bind (typically MAV_COMP_ID_CAMERA).
 */
void mavcam_init(uint8_t system_id, uint8_t component_id);

/**
 * @brief Feed incoming raw data arrays directly into the internal protocol parsing engine.
 * @details Invoke this function instantly within your network loop whenever raw bytes are received from the connection socket.
 * @param[in] data Pointer to the continuous incoming raw byte buffer block chunk.
 * @param[in] data_size Total size of the received chunk payload package in bytes.
 * @param[in] utc Current system UTC timestamp in microseconds, provided by the host platform application.
 */
void mavcam_update(const uint8_t *data, size_t data_size, uint32_t boot_ms, uint64_t utc_us);

/**
 * @brief Background routine handler to maintain background scheduling intervals.
 * @details Continuously invoke this method inside your main application thread loop execution sequence.
 * @param[in] boot_ms Current system uptime timestamp in milliseconds, provided by the host platform application.
 * @param[in] utc Current system UTC timestamp in microseconds, provided by the host platform application.
 */
void mavcam_spin(uint32_t boot_ms, uint64_t utc_us);

/**@brief Configure camera specification for core static camera or attached non-mavlink camera.
 * @param[in] vendor_name Read-only vendor string name representation.
 * @param[in] model_name Read-only hardware model variant string name reference.
 * @param[in] firmware_version Version tracker integer flag metadata.
 * @param[in] focal_length Focal length scaled in millimeters.
 * @param[in] sensor_size_h Horizontal physical plane grid layer scale dimension parameters in millimeters.
 * @param[in] sensor_size_v Vertical physical plane grid layer scale dimension parameters in millimeters.
 * @param[in] resolution_h Maximum active native matrix width specification pixel resolution.
 * @param[in] resolution_v Maximum active native matrix height specification pixel resolution.
 * @param[in] lens_id Reserved for a lens ID. Use 0 if not known.
 * @param[in] flags Standard operational protocol capability support configuration bitmasks.
 * @param[in] cam_def_ver Camera definition version (iteration). Use 0 if not known.
 * @param[in] cam_def_uri Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file). The string needs to be zero terminated. Use a zero-length string if not known.
 * @param[in] gimbal_id Gimbal id of a gimbal associated with this camera. This is the component id of the gimbal device, or 1-6 for non mavlink gimbals. Use 0 if no gimbal is associated with the camera.
 * @param[in] camera_id (It is not supported by this lib) Camera id of a non-MAVLink camera attached to an autopilot (1-6). 0 if the component is a MAVLink camera (with its own component id).
 */
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
                                    uint8_t camera_id);

/**@brief Register a pre-configured video stream channel mapping profile inside the protocol tracking banks.
 * @param[in] stream_id Unique active network channel identifier token index.
 * @param[in] type Stream transmission classification type profile (e.g. 1 for RTSP).
 * @param[in] flags Custom parameter stream functionality attribute support capability bitmasks.
 * @param[in] framerate Capture cycle execution rate frame pacing metrics in Hz.
 * @param[in] resolution_h Active horizontal layout output canvas block coordinate scale resolution pixels.
 * @param[in] resolution_v Active vertical layout output canvas block coordinate scale resolution pixels.
 * @param[in] bitrate Stream network payload bit rate processing constraint density in bits per second.
 * @param[in] rotation Internal image matrix sensor transform layout rotation correction scaling factor degrees.
 * @param[in] hfov Horizontal operational sensor field of view capture window scale metric degrees.
 * @param[in] encoding Stream encoding standard designation flag indicators (e.g. 2 for H265).
 * @param[in] name Descriptive alias string name tag mapped to this connection channel route.
 * @param[in] uri Target absolute string connection network resource unified identification endpoint pointer path.
 * @param[in] camera_id (It is not supported by this lib) Camera id of a non-MAVLink camera attached to an autopilot (1-6). 0 if the component is a MAVLink camera (with its own component id).*/
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
                                uint8_t camera_id);

/**
 * @brief Register a pre-configured storage device profile inside the protocol tracking banks.
 * @param[in] storage_id Unique active block device slot identifier index (typically starts from 1).
 * @param[in] type Storage type classification designation specification (e.g. 3 for SD Card).
 * @param[in] total_capacity Total storage space capacity density processing constraint scaled in Mebibytes (MiB).
 * @param[in] name Descriptive alias string name tag mapped to this storage volume interface path. No more 32 chars
 * @param[in] total_images Total images count on this storage
 * @param[in] read_speed Read speed in MiB/s
 * @param[in] write_speed Write speed in MiB/s
 * @param[in] storage_usage Flags indicating whether this instance is preferred storage for photos, videos, etc.
 */
void mavcam_add_storage(    uint8_t storage_id,
                            uint8_t type,
                            float total_capacity,
                            const char *name,
                            uint32_t total_images,
                            float read_speed,
                            float write_speed,
                            uint8_t storage_usage);

/**
 * @brief Select and activate a specific registered storage device context within the library.
 * @details This function switches the library's active partition focus to the specified device ID
 *          and synchronizes its internal photo sequence indexing engine with the provided file count metrics.
 *          Subsequent snapshot captures and telemetry logging operations will be evaluated against this volume.
 * @param[in] storage_id Unique block device slot identifier index to be marked as active.
 * @param[in] total_images Current number of successfully stored image files discovered on the target volume.
 */
void mavcam_set_current_storage(uint8_t storage_id, uint32_t total_images);

/**
 * @brief Retrieve the static camera specifications block currently loaded in the library context.
 * @details This function provides read access to the master camera descriptive fields (vendor, 
 *          model, lens geometry, and resolution configuration data).
 * @return A copy of the populated MAVLink camera information structure layer.
 */
mavlink_camera_information_t mavcam_get_camera_information(void);

/**
 * @brief Retrieve configuration profiles for a specific registered video broadcast channel.
 * @details Looks up the requested video entity within the internal banks using its identifier. 
 *          If the specified channel index is not configured, it returns a safe zeroed fallback fallback framework.
 * @param[in] stream_id Unique active network channel identifier token index.
 * @return A copy of the populated MAVLink video stream information metrics mapped to this channel.
 */
mavlink_video_stream_information_t mavcam_get_video_stream_information(uint8_t stream_id);

/**
 * @brief Retrieve configuration specifications for a specific registered block storage interface.
 * @details Looks up the targeted storage volume track within the device banks using its hardware allocation slot index. 
 *          If the specified channel index is not configured, it returns a safe zeroed fallback framework.
 * @param[in] storage_id Unique active block device slot identifier index.
 * @return A copy of the populated MAVLink storage information parameters mapped to this slot.
 */
mavlink_storage_information_t mavcam_get_storage_information(uint8_t storage_id);

/**
 * @brief Get the total number of video stream channel pipelines currently added to the library ecosystem.
 * @return Integer tally representing the total registration density of stream endpoints.
 */
uint8_t mavcam_get_video_stream_count(void);

/**
 * @brief Get the total number of hardware storage partitions currently added to the library banks.
 * @return Integer tally representing the total registration density of storage device blocks.
 */
uint8_t mavcam_get_storage_count(void);

/**
 * @brief Get the active MAVLink Component ID bound to this camera protocol stack instance.
 * @details Returns the active execution component identifier token (e.g., MAV_COMP_ID_CAMERA / 100) 
 *          utilized by the library context to sign outbound packet frame headers.
 * @return Unsigned 8-bit integer representing the active library Component ID.
 */
uint8_t mavcam_get_compid(void);

/**
 * @brief Get the monitored or adaptively learned MAVLink System ID of the automated vehicle ecosystem.
 * @details Returns the operational vehicle tracking identifier index. This value might either be 
 *          explicitly forced during initialization or dynamically parsed and latched from inbound autopilot heartbeats.
 * @return Unsigned 8-bit integer representing the tracked drone System ID.
 */
uint8_t mavcam_get_sysid(void);


#endif // MAVCAM_H