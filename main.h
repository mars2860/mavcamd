#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

#define APP_VERSION "v1"

/// Default absolute filesystem path to the JSON camera configuration file.
#define DEFAULT_CONFIG_PATH                 "/etc/mavcam.json"

/// Default loopback network IP address and UDP port interface for communicating with mavlink-router.
#define DEFAULT_ROUTER_ADDRESS              "127.0.0.1:14553"

/// Unified naming format for generating new image filenames.
#define CAMERA_IMAGE_NAME_WRITE_FMT         "IMG_%06d.jpg"

/// Unified naming prefix used for scanning and matching image files on disk.
#define CAMERA_IMAGE_NAME_PREFIX_FMT        "IMG_%06d"

/// Unified format pattern used by sscanf to parse sequence indexes from filenames.
#define CAMERA_IMAGE_NAME_PARSE_FMT         "IMG_%6d"

/// Absolute filesystem path to the Waybeam video encoder configuration file.
#define WAYBEAM_CONFIG_PATH                 "/etc/waybeam.json"

// Time difference threshold in milliseconds to trigger system clock adjustment.
#define SYSTEM_TIME_SYNC_THRESHOLD_MS       3000

#define CURRENT_STORAGE_ID      1

typedef enum {
    VENC_UNKNOWN=0,
    VENC_WAYBEAM=1
} venc_t;

/// Structure to store VENC REST API endpoints
typedef struct {
    char *venc;
    char *base_url;
    char *video_start;
    char *video_stop;
    char *video_status;
    char *photo_capture;
    char *set_video_mode;
    char *set_photo_mode;
} rest_api_t;

/// Structure to handle dynamic memory allocation for incoming curl responses
typedef struct {
    char *response_data;
    size_t size;
} curl_response_buffer_t;

/// Structure for storage
typedef struct {
    uint8_t id;
    char dcim[255];
    uint32_t image_count;
    uint32_t last_image_idx;
} storage_t;

#endif // MAIN_H
