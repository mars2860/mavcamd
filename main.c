#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>
#include <dirent.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <curl/curl.h>
#include <math.h>
#include "mavcam.h"
#include "main.h"

char *camera_config_path = NULL;
char *router_arg = NULL;
char router_ip[64] = {0};
int router_port = 0;

int sock_fd = -1;
struct sockaddr_in router_addr;

rest_api_t rest_api;
venc_t venc = VENC_UNKNOWN;

storage_t storage[MAX_STORAGES];
static int storage_count = 0;
static int current_storage_id = CURRENT_STORAGE_ID;

static bool enable_temperature_wifilink2 = false;

//-----------------------------------------------------------------------------

static uint32_t get_time_boot_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

uint64_t get_current_utc_us(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return 0;
    }
    return ((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL);
}

// Helper function to recursively create a directory path (like mkdir -p)
static int mkdir_p(const char *path, mode_t mode) {
    char tmp[256];
    char *p = NULL;
    size_t len;

    snprintf(tmp, sizeof(tmp), "%s", path);
    len = strlen(tmp);
    if (len == 0) {
        return -1;
    }

    // Remove trailing slash if any
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = '\0';
    }

    // Iterate through the path string to create nested subdirectories
    for (p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            // Check if directory exists, if not, create it
            if (mkdir(tmp, mode) != 0 && errno != EEXIST) {
                return -1;
            }
            *p = '/';
        }
    }

    // Create the final trailing directory component
    if (mkdir(tmp, mode) != 0 && errno != EEXIST) {
        return -1;
    }

    return 0;
}

// Helper function to allocate memory and read an entire file into a string
static char *read_file_to_string(const char *path) {
    FILE *file = fopen(path, "r");
    if (!file) return NULL;

    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);

    char *buffer = malloc(length + 1);
    if (buffer) {
        size_t read_bytes = fread(buffer, 1, length, file);
        buffer[read_bytes] = '\0';
    }
    fclose(file);
    return buffer;
}

// Ensure the target storage folder for photo artifacts exists on the platform filesystem
static void ensure_storage_directory(const char *path) {
    LOG_INFO("Checking image storage directory: %s", path);
    
    // 0755 provides read/write/execute for owner, and read/execute for group/others
    if (mkdir_p(path, 0755) == 0) {
        LOG_INFO("Image storage directory is ready");
    } else {
        LOG_ERROR("Failed to create storage directory '%s': %s", path, strerror(errno));
    }
}


static void init_storages(void) {
    memset(storage,0,sizeof(storage));
}

static storage_t *add_storage(  uint8_t storage_id,
                                const char *dcim,
                                uint32_t image_count,
                                uint32_t last_image_idx) {
    if(storage_count >= MAX_STORAGES) 
        return NULL;

    storage[storage_count].id = storage_id;
    storage[storage_count].image_count = image_count;
    storage[storage_count].last_image_idx = last_image_idx;

    if(dcim) {
        strncpy(    storage[storage_count].dcim,
                    dcim,
                    sizeof(storage[storage_count].dcim));

        ensure_storage_directory(dcim);
    }

    storage_count++;

    return &storage[storage_count];
}

static storage_t *get_storage(uint8_t storage_id) {
    for(int i = 0; i < MAX_STORAGES; i++) {
        if(storage[i].id == storage_id) {
            return &storage[i];
        }
    }

    return NULL;
}

static void init_rest_api(void) {
    memset(&rest_api,0,sizeof(rest_api));
}

static void free_rest_api(void) {
    if(rest_api.venc != NULL) {
        free(rest_api.venc);
        rest_api.venc = NULL;
    }

    if(rest_api.base_url != NULL) {
        free(rest_api.base_url);
        rest_api.base_url = NULL;
    }

    if(rest_api.video_start != NULL) {
        free(rest_api.video_start);
        rest_api.video_start = NULL;
    }

    if(rest_api.video_stop != NULL) {
        free(rest_api.video_stop);
        rest_api.video_stop = NULL;
    }

    if(rest_api.video_status != NULL) {
        free(rest_api.video_status);
        rest_api.video_status = NULL;
    }

    if(rest_api.photo_capture != NULL) {
        free(rest_api.photo_capture);
        rest_api.photo_capture = NULL;
    }

    if(rest_api.set_video_mode != NULL) {
        free(rest_api.set_video_mode);
        rest_api.set_video_mode = NULL;
    }

    if(rest_api.set_photo_mode != NULL) {
        free(rest_api.set_photo_mode);
        rest_api.set_photo_mode = NULL;
    }
}

static int init_network(void) {
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        LOG_ERROR("Socket creation failed");
        return EXIT_FAILURE;
    }

    // Configure non-blocking network timeout for the main loop
    struct timeval tv = {.tv_sec = 0, .tv_usec = 100000}; // 100ms timeout
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        LOG_ERROR("Setting socket timeout failed");
        close(sock_fd);
        return EXIT_FAILURE;
    }

    memset(&router_addr, 0, sizeof(router_addr));
    router_addr.sin_family = AF_INET;
    router_addr.sin_port = htons(router_port);
    if (inet_pton(AF_INET, router_ip, &router_addr.sin_addr) <= 0) {
        LOG_ERROR("Invalid router IP address: %s", router_ip);
        close(sock_fd);
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

static void free_network(void) {
    close(sock_fd);
}


static uint32_t get_total_images(const char *dcim) {
    if (!dcim || strlen(dcim) == 0) {
        LOG_WARN("Failed to count total images: Target path descriptor is empty or null.");
        return 0;
    }

    DIR *dir = opendir(dcim);
    if (!dir) {
        LOG_WARN("Cannot open storage layout directory for imagery verification path: %s", dcim);
        return 0; 
    }

    struct dirent *entry;
    uint32_t meta_file_count = 0;

    // Read all filesystem storage artifacts within the opened folder
    while ((entry = readdir(dir)) != NULL) {
        // Enforce structural validation matching predictable prefix tokens
        if (strncmp(entry->d_name, "IMG_", 4) == 0) {
            const char *dot_ext = strrchr(entry->d_name, '.');
            
            // Check if the extracted dot component strictly matches the metadata marker suffix
            if (dot_ext && strcmp(dot_ext, ".meta") == 0) {
                meta_file_count++;
            }
        }
    }
    
    closedir(dir);

    return meta_file_count;
}

// Scan the image storage directory at boot to discover the highest existing image index
static uint32_t find_last_image_index(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        // If directory cannot be opened, assume starting sequence from scratch
        return 0; 
    }

    struct dirent *entry;
    int32_t max_index = 0;

    while ((entry = readdir(dir)) != NULL) {
        // Look for metadata files starting with the predictable "IMG_" prefix
        if (strncmp(entry->d_name, "IMG_", 4) == 0 && strstr(entry->d_name, ".meta") != NULL) {
            int32_t current_idx = 0;
            
            // Extract the integer index right after "IMG_" using sscanf
            if (sscanf(entry->d_name, CAMERA_IMAGE_NAME_PARSE_FMT, &current_idx) == 1) {
                if (current_idx > max_index) {
                    max_index = current_idx;
                }
            }
        }
    }
    
    closedir(dir);

    return (uint32_t)max_index;
}

// Parses the frame size string from waybeam configuration into numeric dimensions.
// Supports both naming presets (e.g., "1080p", "4k") and direct "WxH" formats (e.g., "1920x1080").
// size_str: Raw size string from JSON.
// w: Pointer to store detected horizontal resolution.
// h: Pointer to store detected vertical resolution.
static void parse_waybeam_size(const char *size_str, uint16_t *w, uint16_t *h) {
    if (!size_str) {
        *w = 1920; *h = 1080; // Default fallback standard
        return;
    }

    // First, try parsing as a direct "WidthxHeight" numeric format string
    unsigned int width_val = 0;
    unsigned int height_val = 0;
    if (sscanf(size_str, "%ux%u", &width_val, &height_val) == 2) {
        if (width_val > 0 && height_val > 0) {
            *w = (uint16_t)width_val;
            *h = (uint16_t)height_val;
            return;
        }
    }

    // Fallback to standard industry naming layout presets
    if (strcmp(size_str, "1080p") == 0) {
        *w = 1920; *h = 1080;
    } else if (strcmp(size_str, "720p") == 0) {
        *w = 1280; *h = 720;
    } else if (strcmp(size_str, "4k") == 0 || strcmp(size_str, "2160p") == 0) {
        *w = 3840; *h = 2160;
    } else if (strcmp(size_str, "2k") == 0 || strcmp(size_str, "1440p") == 0) {
        *w = 2560; *h = 1440;
    } else {
        // Default baseline fallback if parsing pattern mismatch occurs
        *w = 1920; *h = 1080;
    }
}

// Extracts the network port identifier suffix from the Waybeam target server URI string.
// server_url: Raw network endpoint tracking string (e.g., "udp://192.168.0.10:5600").
// port_out: Output buffer to store parsed destination port string.
// max_len: Maximum byte size allocated for the port output buffer container.
static void parse_waybeam_port(const char *server_url, char *port_out, size_t max_len) {
    if (!server_url) {
        snprintf(port_out, max_len, "5600");
        return;
    }
    
    // Isolate the port substring sequence tracked after the trailing colon delimiter
    const char *colon = strrchr(server_url, ':');
    if (colon && *(colon + 1) != '\0') {
        snprintf(port_out, max_len, "%s", colon + 1);
    } else {
        snprintf(port_out, max_len, "5600");
    }
}

// Main parser function for mavcam.json config
static int load_config(const char *path) {
    char *json_data = read_file_to_string(path);
    if (!json_data) {
        LOG_ERROR("Cannot open config file '%s'", path);
        return -1;
    }

    cJSON *root = cJSON_Parse(json_data);
    if (!root) {
        LOG_ERROR("JSON parse error before: [%s]", cJSON_GetErrorPtr());
        free(json_data);
        return -1;
    }

    // 1. Parse camera_information object
    cJSON *cam_json = cJSON_GetObjectItemCaseSensitive(root, "camera_information");
    if (cam_json) {
        int mavcam_compid = 100;
        char vendor_name[32] = {0};
        char model_name[32] = {0};
        int firmware = 0;
        double focal_length = 0;
        double sensor_size_h = 0;
        double sensor_size_v = 0;
        int resolution_h = 0;
        int resolution_v = 0;
        int lens_id = 0;
        int flags = 0;
        int cam_def_ver = 0;
        char cam_def_uri[140] = {0};
        int gimbal_device_id = 0;
        int camera_device_id = 0;

        cJSON *j_comp_id = cJSON_GetObjectItemCaseSensitive(cam_json, "component_id");
        cJSON *j_vendor = cJSON_GetObjectItemCaseSensitive(cam_json, "vendor_name");
        cJSON *j_model = cJSON_GetObjectItemCaseSensitive(cam_json, "model_name");
        cJSON *j_firmware = cJSON_GetObjectItemCaseSensitive(cam_json, "firmware_version");
        cJSON *j_focal_length = cJSON_GetObjectItemCaseSensitive(cam_json, "focal_length");
        cJSON *j_sensor_size_h = cJSON_GetObjectItemCaseSensitive(cam_json, "sensor_size_h");
        cJSON *j_sensor_size_v = cJSON_GetObjectItemCaseSensitive(cam_json, "sensor_size_v");
        cJSON *j_resolution_h = cJSON_GetObjectItemCaseSensitive(cam_json, "resolution_h");
        cJSON *j_resolution_v = cJSON_GetObjectItemCaseSensitive(cam_json, "resolution_v");
        cJSON *j_lens_id = cJSON_GetObjectItemCaseSensitive(cam_json, "lens_id");
        cJSON *j_flags = cJSON_GetObjectItemCaseSensitive(cam_json, "flags");
        cJSON *j_cam_def_ver = cJSON_GetObjectItemCaseSensitive(cam_json, "cam_definition_version");
        cJSON *j_cam_def_uri = cJSON_GetObjectItemCaseSensitive(cam_json, "cam_definition_uri");
        cJSON *j_gimbal_device_id = cJSON_GetObjectItemCaseSensitive(cam_json, "gimbal_device_id");
        cJSON *j_camera_device_id = cJSON_GetObjectItemCaseSensitive(cam_json, "camera_device_id");

        if(j_comp_id) mavcam_compid = j_comp_id->valueint;

        if (j_vendor && j_vendor->valuestring) {
            strncpy(vendor_name, j_vendor->valuestring, sizeof(vendor_name) - 1);
        } else {
            strncpy(vendor_name, "Unknown", sizeof(vendor_name) - 1);
        }

        if (j_model && j_model->valuestring) {
            strncpy(model_name, j_model->valuestring, sizeof(model_name) - 1);
        } else {
            strncpy(model_name, "Unknown", sizeof(model_name) - 1);
        }

        if(j_firmware) firmware = j_firmware->valueint;
        if(j_focal_length) focal_length = j_focal_length->valuedouble;
        if(j_sensor_size_h) sensor_size_h = j_sensor_size_h->valuedouble;
        if(j_sensor_size_v) sensor_size_v = j_sensor_size_v->valuedouble;
        if(j_resolution_h) resolution_h = j_resolution_h->valueint;
        if(j_resolution_v) resolution_v = j_resolution_v->valueint;
        if(j_lens_id) lens_id = j_lens_id->valueint;
        if(j_flags) flags = j_flags->valueint;
        if(j_cam_def_ver) cam_def_ver = j_cam_def_ver->valueint;

        if (j_cam_def_uri && j_cam_def_uri->valuestring) {
            strncpy(cam_def_uri, j_cam_def_uri->valuestring, sizeof(cam_def_uri) - 1);
        }

        if(j_gimbal_device_id) gimbal_device_id = j_gimbal_device_id->valueint;
        if(j_camera_device_id) camera_device_id = j_camera_device_id->valueint;

        mavcam_init(0, mavcam_compid);

        mavcam_set_camera_information(  vendor_name,
                                        model_name,
                                        firmware,
                                        focal_length,
                                        sensor_size_h,
                                        sensor_size_v,
                                        resolution_h,
                                        resolution_v,
                                        lens_id,
                                        flags,
                                        cam_def_ver,
                                        cam_def_uri,
                                        gimbal_device_id,
                                        camera_device_id);
    } else {
        LOG_ERROR("There is no mandatory camera_information object in config file");
        return -1;
    }

    // 2. Parse video_streams array and dynamically calculate total stream count
    cJSON *streams_array = cJSON_GetObjectItemCaseSensitive(root, "video_streams");
    if (streams_array && cJSON_IsArray(streams_array)) {
        int video_stream_count = cJSON_GetArraySize(streams_array);
        if (video_stream_count > MAX_STREAMS) video_stream_count = MAX_STREAMS;

        for (int i = 0; i < video_stream_count; i++) {
            cJSON *item = cJSON_GetArrayItem(streams_array, i);
            
            if(!item)
                continue;

            int stream_id = 1;
            int type = 0;
            int flags = 0;
            double framerate = 0;
            int resolution_h = 0;
            int resolution_v = 0;
            int bitrate = 0;
            int rotation = 0;
            int hfov = 0;
            char name[32] = {0};
            char uri[160] = {0};
            int encoding = 0;
            int camera_device_id = 0;

            cJSON* j_stream_id = cJSON_GetObjectItemCaseSensitive(item, "stream_id");
            cJSON* j_type = cJSON_GetObjectItemCaseSensitive(item, "type");
            cJSON* j_flags = cJSON_GetObjectItemCaseSensitive(item, "flags");
            cJSON* j_framerate = cJSON_GetObjectItemCaseSensitive(item, "framerate");
            cJSON* j_resolution_h = cJSON_GetObjectItemCaseSensitive(item, "resolution_h");
            cJSON* j_resolution_v = cJSON_GetObjectItemCaseSensitive(item, "resolution_v");
            cJSON* j_bitrate = cJSON_GetObjectItemCaseSensitive(item, "bitrate");
            cJSON* j_rotation = cJSON_GetObjectItemCaseSensitive(item, "rotation");
            cJSON* j_hfov = cJSON_GetObjectItemCaseSensitive(item, "hfov");
            cJSON* j_encoding = cJSON_GetObjectItemCaseSensitive(item, "encoding");
            cJSON* j_camera_device_id = cJSON_GetObjectItemCaseSensitive(item, "camera_device_id");
            cJSON *j_name = cJSON_GetObjectItemCaseSensitive(item, "name");
            cJSON *j_uri = cJSON_GetObjectItemCaseSensitive(item, "uri");

            if(j_stream_id) stream_id = j_stream_id->valueint;
            if(j_type) type = j_type->valueint;
            if(j_flags) flags = j_flags->valueint;
            if(j_framerate) framerate = j_framerate->valuedouble;
            if(j_resolution_h) resolution_h = j_resolution_h->valueint;
            if(j_resolution_v) resolution_v = j_resolution_v->valueint;
            if(j_bitrate) bitrate = j_bitrate->valueint;
            if(j_rotation) rotation = j_rotation->valueint;
            if(j_hfov) hfov = j_hfov->valueint;
            if(j_encoding) encoding = j_encoding->valueint;
            if(j_camera_device_id) camera_device_id = j_camera_device_id->valueint;
            
            if (j_name && j_name->valuestring) {
                strncpy(name, j_name->valuestring, sizeof(name) - 1);
            }

            if (j_uri && j_uri->valuestring) {
                strncpy(uri, j_uri->valuestring, sizeof(uri) - 1);
            }

            mavcam_add_video_stream(    stream_id,
                                        type,
                                        flags,
                                        framerate,
                                        resolution_h,
                                        resolution_v,
                                        bitrate,
                                        rotation,
                                        hfov,
                                        encoding,
                                        name,
                                        uri,
                                        camera_device_id);
        }
    }

    // 3. Parse local rest_api settings object
    cJSON *api_json = cJSON_GetObjectItemCaseSensitive(root, "rest_api");
    if (api_json) {
        cJSON *v_enc = cJSON_GetObjectItemCaseSensitive(api_json, "venc");
        cJSON *base = cJSON_GetObjectItemCaseSensitive(api_json, "base_url");
        cJSON *v_start = cJSON_GetObjectItemCaseSensitive(api_json, "video_start");
        cJSON *v_stop = cJSON_GetObjectItemCaseSensitive(api_json, "video_stop");
        cJSON *v_status = cJSON_GetObjectItemCaseSensitive(api_json, "video_status");
        cJSON *p_cap = cJSON_GetObjectItemCaseSensitive(api_json, "photo_capture");
        cJSON *s_v_mode = cJSON_GetObjectItemCaseSensitive(api_json, "set_video_mode");
        cJSON *s_p_mode = cJSON_GetObjectItemCaseSensitive(api_json, "set_photo_mode");

        free_rest_api(); // it is needed if we will reload config

        if (v_enc && v_enc->valuestring) rest_api.venc = strdup(v_enc->valuestring);
        if (base && base->valuestring) rest_api.base_url = strdup(base->valuestring);
        if (v_start && v_start->valuestring) rest_api.video_start = strdup(v_start->valuestring);
        if (v_stop && v_stop->valuestring) rest_api.video_stop = strdup(v_stop->valuestring);
        if (v_status && v_status->valuestring) rest_api.video_status = strdup(v_status->valuestring);
        if (p_cap && p_cap->valuestring) rest_api.photo_capture = strdup(p_cap->valuestring);
        if (s_v_mode && s_v_mode->valuestring) rest_api.set_video_mode = strdup(s_v_mode->valuestring);
        if (s_p_mode && s_p_mode->valuestring) rest_api.set_photo_mode = strdup(s_p_mode->valuestring);

        venc = VENC_UNKNOWN;

        if(rest_api.venc && strcmp(rest_api.venc,"waybeam") == 0) {
            venc = VENC_WAYBEAM;
        }
    }

    // 4. Parse storage_information object
    cJSON *storages_array = cJSON_GetObjectItemCaseSensitive(root, "storages");
    if (storages_array && cJSON_IsArray(storages_array)) {
        int storages_count = cJSON_GetArraySize(storages_array);
        if (storages_count > MAX_STORAGES) storages_count = MAX_STORAGES;

        for (int i = 0; i < storages_count; i++) {
            cJSON *item = cJSON_GetArrayItem(storages_array, i);
            
            if(!item)
                continue;

            int storage_id = 1;
            int type = 0;
            int storage_usage = 0;
            char name[32] = {0};
            char dcim[160] = {0};
            double read_speed = 0;
            double write_speed = 0;
            float total_mib = 0.f;
            float used_mib = 0.f;
            float available_mib = 0.f;
            int total_images = 0;
            int last_image_idx = 0;

            cJSON* j_storage_id = cJSON_GetObjectItemCaseSensitive(item, "storage_id");
            cJSON* j_type = cJSON_GetObjectItemCaseSensitive(item, "type");
            cJSON* j_storage_usage = cJSON_GetObjectItemCaseSensitive(item, "storage_usage");
            cJSON* j_write_speed = cJSON_GetObjectItemCaseSensitive(item, "write_speed");
            cJSON* j_read_speed = cJSON_GetObjectItemCaseSensitive(item, "read_speed");
            cJSON* j_name = cJSON_GetObjectItemCaseSensitive(item, "name");
            cJSON* j_dcim = cJSON_GetObjectItemCaseSensitive(item, "dcim");

            if(j_storage_id) storage_id = j_storage_id->valueint;
            if(j_type) type = j_type->valueint;
            if(j_storage_usage) storage_usage = j_storage_usage->valueint;
            if(j_write_speed) write_speed = j_write_speed->valuedouble;
            if(j_read_speed) read_speed = j_read_speed->valuedouble;

            if (j_name && j_name->valuestring) {
                strncpy(name, j_name->valuestring, sizeof(name) - 1);
            }

            if (j_dcim && j_dcim->valuestring) {
                strncpy(dcim, j_dcim->valuestring, sizeof(dcim) - 1);
                last_image_idx = find_last_image_index(dcim);
                total_images = get_total_images(dcim);
            }

            add_storage(storage_id, dcim, total_images, last_image_idx);
            
            mavcam_cb_query_storage_metrics(storage_id, &used_mib, &available_mib, &total_mib);

            mavcam_add_storage(
                storage_id,
                type,
                total_mib,
                name,
                total_images,
                read_speed,
                write_speed,
                storage_usage);

            if(storage_id == current_storage_id) {
                mavcam_set_current_storage(current_storage_id, total_images);
                LOG_INFO("Recovered last image sequence index from disk: %d, Total Images %d, Storage ID %d", 
                    last_image_idx,total_images,current_storage_id);
            }
        }
    }

    // 5. Parse waybeam config
    // Check if any video streams were statically loaded from mavcam.json.
    // If the count is 0 and the active encoder is Waybeam, trigger automated configuration discovery.
    if (venc == VENC_WAYBEAM && mavcam_get_video_stream_count() == 0) {
        if (access(WAYBEAM_CONFIG_PATH, F_OK) == 0) {
            LOG_INFO("No video streams defined in mavcam.json. Parsing auto-configuration from %s", WAYBEAM_CONFIG_PATH);
            
            FILE *w_file = fopen(WAYBEAM_CONFIG_PATH, "r");
            if (w_file) {
                fseek(w_file, 0, SEEK_END);
                long w_size = ftell(w_file);
                fseek(w_file, 0, SEEK_SET);

                char *w_buf = malloc(w_size + 1);
                if (w_buf) {
                    size_t read_bytes = fread(w_buf, 1, w_size, w_file);
                    w_buf[read_bytes] = '\0';

                    cJSON *w_root = cJSON_Parse(w_buf);
                    if (w_root) {
                        // Extract required configuration branches from waybeam structure layer
                        cJSON *video0 = cJSON_GetObjectItemCaseSensitive(w_root, "video0");
                        cJSON *outgoing = cJSON_GetObjectItemCaseSensitive(w_root, "outgoing");

                        if (video0) {
                            cJSON *fps_item = cJSON_GetObjectItemCaseSensitive(video0, "fps");
                            cJSON *size_item = cJSON_GetObjectItemCaseSensitive(video0, "size");
                            cJSON *bitrate_item = cJSON_GetObjectItemCaseSensitive(video0, "bitrate");
                            cJSON *server_item = outgoing ? cJSON_GetObjectItemCaseSensitive(outgoing, "server") : NULL;

                            // Initialize pipeline telemetry properties from hardware definition nodes
                            float fps = fps_item ? (float)fps_item->valuedouble : 60.0f;
                            uint32_t bitrate = bitrate_item ? (uint32_t)bitrate_item->valueint : 12288;
                            
                            uint16_t res_h = 1920;
                            uint16_t res_v = 1080;
                            if (size_item && size_item->valuestring) {
                                parse_waybeam_size(size_item->valuestring, &res_h, &res_v);
                            }

                            char port_str[16];
                            parse_waybeam_port(server_item ? server_item->valuestring : NULL, port_str, sizeof(port_str));

                            LOG_INFO("Auto-detected Waybeam Stream: %dx%d @ %.1f FPS, Bitrate: %u kbps, Port: %s", 
                                        res_h, res_v, fps, bitrate, port_str);
                                
                            char name_str[32];
                            snprintf(name_str,sizeof(name_str),"waybeam_%dx%d",res_h,res_v);

                            // Register telemetry channel properties inside active MAVLink stream tracking table
                            mavcam_add_video_stream( 1, // stream_id
                                                     VIDEO_STREAM_TYPE_RTPUDP, // waybeam can stream only RTP-UDP
                                                     VIDEO_STREAM_STATUS_FLAGS_RUNNING, // flags
                                                     fps,
                                                     res_h,
                                                     res_v,
                                                     bitrate*1000,  // bitrate has to be in bit/s
                                                     0,
                                                     71,  // hfov
                                                     VIDEO_STREAM_ENCODING_H265,  // waybeam can encode only H265
                                                     "main_waybeam", // name
                                                     port_str, // uri,
                                                     0 // camera_id
                                                     );
                        } else {
                            LOG_ERROR("Invalid waybeam.json format: 'video0' section is missing.");
                        }
                        cJSON_Delete(w_root);
                    } else {
                        LOG_ERROR("Failed to parse cJSON from %s", WAYBEAM_CONFIG_PATH);
                    }
                    free(w_buf);
                }
                fclose(w_file);
            }
        } else {
            LOG_WARN("Waybeam integration active, but %s does not exist.", WAYBEAM_CONFIG_PATH);
        }
    }

    cJSON_Delete(root);
    free(json_data);

    return 0;
}

// Callback function for libcurl to discard incoming payload data without saving it to memory
static size_t async_discard_cb(void *ptr, size_t size, size_t nmemb, void *data) {
    (void)ptr;
    (void)data;
    return size * nmemb; // Tell curl that all bytes were successfully "processed"
}

// Callback function for libcurl to write incoming HTTP stream data into dynamic memory
static size_t mavcam_curl_write_cb(void *data, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    curl_response_buffer_t *mem = (curl_response_buffer_t *)userp;

    char *ptr = realloc(mem->response_data, mem->size + realsize + 1);
    if (!ptr) {
        LOG_ERROR("Not enough memory for curl buffer realloc");
        return 0; // Out of memory
    }

    mem->response_data = ptr;
    memcpy(&(mem->response_data[mem->size]), data, realsize);
    mem->size += realsize;
    mem->response_data[mem->size] = 0; // Null-terminate the string

    return realsize;
}

// Callback function for libcurl to dump incoming binary JPEG streams directly into a file descriptor
static size_t mavcam_curl_file_write_cb(void *ptr, size_t size, size_t nmemb, void *stream) {
    return fwrite(ptr, size, nmemb, (FILE *)stream);
}

// Perform a REST API request to download a snapshot and write it directly onto disk storage
static bool download_and_save_photo(const char *endpoint, const char *local_filename) {
    if (!rest_api.base_url || !endpoint || !local_filename) return false;

    bool success = false;
    CURL *curl = curl_easy_init();
    if (curl) {
        char full_url[512];
        snprintf(full_url, sizeof(full_url), "%s%s", rest_api.base_url, endpoint);

        // Open binary file pointer for writing the payload directly to disk
        FILE *fp = fopen(local_filename, "wb");
        if (!fp) {
            LOG_ERROR("Failed to open local file for writing: %s", local_filename);
            curl_easy_cleanup(curl);
            return false;
        }

        LOG_DEBUG("Downloading photo from: %s", full_url);

        curl_easy_setopt(curl, CURLOPT_URL, full_url);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Allow up to 10 seconds for large files
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mavcam_curl_file_write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);

        CURLcode res = curl_easy_perform(curl);
        fclose(fp); // Ensure file descriptor handles are released immediately

        if (res != CURLE_OK) {
            LOG_ERROR("REST API photo download failed: %s", curl_easy_strerror(res));
            remove(local_filename); // Purge corrupted or incomplete file chunks
        } else {
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

            if (http_code == 200) {
                LOG_DEBUG("Photo downloaded and written successfully (HTTP 200 OK)");
                success = true;
            } else {
                LOG_ERROR("REST API photo source returned HTTP code %ld", http_code);
                remove(local_filename);
            }
        }
        curl_easy_cleanup(curl);
    }
    return success;
}

// Perform a REST API request to get current video recording status from Waybeam VENC
static int waybeam_query_status_is_recording(uint8_t stream_id) {
    (void)stream_id;
    if (!rest_api.base_url || !rest_api.video_status) 
        return -1;

    int video_status = -1;

    CURL *curl = curl_easy_init();
    if (curl) {
        char full_url[512];
        snprintf(full_url, sizeof(full_url), "%s%s", rest_api.base_url, rest_api.video_status);

        curl_response_buffer_t chunk = { .response_data = malloc(1), .size = 0 };

        curl_easy_setopt(curl, CURLOPT_URL, full_url);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2); // Prevent blocking the main loop
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mavcam_curl_write_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);

        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK && chunk.size > 0) {
            // Parse incoming JSON data
            cJSON *root = cJSON_Parse(chunk.response_data);
            if (root) {
                cJSON *data_obj = cJSON_GetObjectItemCaseSensitive(root, "data");
                if (data_obj) {
                    cJSON *active_obj = cJSON_GetObjectItemCaseSensitive(data_obj, "active");
                    if (active_obj && cJSON_IsBool(active_obj)) {
                        if (cJSON_IsTrue(active_obj)) {
                            video_status = 1;
                        } else {
                            video_status = 0;
                        }
                    }
                }
                cJSON_Delete(root);
            }
        } else {
            LOG_ERROR("Failed to fetch video status from REST: %s", curl_easy_strerror(res));
        }

        free(chunk.response_data);
        curl_easy_cleanup(curl);
    }

    return video_status;
}

// Universal helper to perform asynchronous libcurl HTTP GET requests to Majestic/Waybeam REST API
// Returns true if HTTP Response Code is 200 OK, otherwise returns false
static bool send_rest_request(const char *endpoint) {
    if (!rest_api.base_url || !endpoint) return false;

    bool success = false;
    CURL *curl = curl_easy_init();
    if (curl) {
        // Construct target URL (base_url + endpoint)
        char full_url[256];
        snprintf(full_url, sizeof(full_url), "%s%s", rest_api.base_url, endpoint);

        LOG_DEBUG("Sending REST request to: %s", full_url);

        curl_easy_setopt(curl, CURLOPT_URL, full_url);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2); // Avoid blocking daemon (2 seconds max)
        
        // Explicitly enforce HTTP GET method and suppress response output to stdout
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, async_discard_cb);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, NULL);

        // Perform request
        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            LOG_ERROR("REST API network request failed: %s", curl_easy_strerror(res));
        } else {
            // Retrieve and validate the real HTTP response code from the server
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

            if (http_code == 200) {
                LOG_DEBUG("REST API triggered successfully (HTTP 200 OK)");
                success = true;
            } else {
                LOG_ERROR("REST API returned HTTP error code %ld for endpoint %s", http_code, endpoint);
            }
        }
        curl_easy_cleanup(curl);
    }
    return success;
}

static void print_usage(void) {
    fprintf(stderr, "MAVLink Camera Protocol Daemon for OpenIPC %s\n\n", APP_VERSION);
    fprintf(stderr, "Usage: mavcamd [options]\n\n");
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -a, --address <ip:port>   IP address and UDP port for mavlink traffic (default: %s)\n",DEFAULT_ROUTER_ADDRESS);
    fprintf(stderr, "  -c, --config  <file>      Path to MAVLink Camera configuration JSON file (default: %s)\n",DEFAULT_CONFIG_PATH);
    fprintf(stderr, "  -t, --temp  <file>        Inject SoC temperature into telemetry(HiSilicon and SigmaStart supported) via RAW_IMU mavlink message\n");
    fprintf(stderr, "  -h, --help                Show this help message\n\n");
    fprintf(stderr, "For example with logging: mavcamd -a 127.0.0.1:14553 > /tmp/mavcamd.log 2>&1\n");
}

// Parse CLI and load config
static int parse_cli(int argc, char *argv[]) {
    static struct option long_options[] = {
        {"address", required_argument, 0, 'a'},
        {"config",  required_argument, 0, 'c'},
        {"temp",    no_argument,       0, 't'},
        {"help",    no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int opt;

    // Set default values before parsing arguments
    camera_config_path = DEFAULT_CONFIG_PATH;
    router_arg = DEFAULT_ROUTER_ADDRESS;

    // Parse incoming CLI arguments
    while ((opt = getopt_long(argc, argv, "a:c:th", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'a': router_arg = optarg; break;
            case 'c': camera_config_path = optarg; break;
            case 't': enable_temperature_wifilink2 = true; break;
            case 'h': print_usage(); return EXIT_FAILURE;
            default: print_usage(); return EXIT_FAILURE;
        }
    }

    // Parse network address parameter in IP:PORT format
    if (sscanf(router_arg, "%63[^:]:%d", router_ip, &router_port) != 2) {
        LOG_ERROR("Invalid address format '%s'. Expected IP:PORT", router_arg);
        return EXIT_FAILURE;
    }

    // Validate the parsed port number range
    if (router_port <= 0 || router_port > 65535) {
        LOG_ERROR("Invalid port number '%d'.", router_port);
        return EXIT_FAILURE;
    }

    LOG_INFO("Target router address parsed: %s:%d", router_ip, router_port);

    // Load and validate the JSON configuration file
    if (load_config(camera_config_path) == 0) {
        mavlink_camera_information_t cam_info = mavcam_get_camera_information();
        int video_stream_count = mavcam_get_video_stream_count();
        int comp_id = mavcam_get_compid();
        LOG_INFO("Config loaded successfully!");
        LOG_INFO("Camera: %s %s, CompID: %d, Flags: %d, Streams detected: %d", 
            cam_info.vendor_name, cam_info.model_name, comp_id, cam_info.flags, video_stream_count);
        switch(venc) {
            case VENC_WAYBEAM:
                LOG_INFO("Venc: OpenIPC Waybeam");
                break;
            default:
                LOG_INFO("Venc: Unknown");
                break;
        }
    } else {
        LOG_ERROR("Failed to initialize configuration. Exiting.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/// @brief
/// @return -100 if no sigmastar found
static int get_temp_sigma_star(void) {
    
    static int last_board_temp = -100;

	// https://wx.comake.online/doc/doc/SigmaStarDocs-SSD220-SIGMASTAR-202305231834/platform/BSP/Ikayaki/frequency_en.html
    const char *sensor_path = "/sys/devices/virtual/mstar/msys/TEMP_R";
	FILE *file = fopen(sensor_path, "r"); // Temperature 62
	if (file == NULL) {
		LOG_ERROR("No temp data at %s", sensor_path);
		return -100;
	}

	char buff[200];
	// Read lines from the file and parse for dropped packets
	if (fgets(buff, sizeof(buff), file) != NULL) {
		// Find "Temperature"
		char *temperature_str = strstr(buff, "Temperature");
		if (temperature_str != NULL)
			// Extract temperature value
			last_board_temp = atoi(temperature_str + 12);
	}

	fclose(file);

	if (last_board_temp < -90)
		LOG_ERROR("No temp data in file %s", sensor_path);

	return last_board_temp;
}

static void send_temp_to_gcs(void) {
    
    static uint64_t last_sent_timestamp = 0;

	// Once a second
	if (get_time_boot_ms() - last_sent_timestamp < 1000) return;

	last_sent_timestamp = get_time_boot_ms();

	// read the temperature only once per second
	int temp = get_temp_sigma_star();

	mavlink_message_t message;
	
    mavlink_msg_raw_imu_pack(
        mavcam_get_sysid(),
        mavcam_get_compid(),
        &message,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, // IMU id       
        temp * 100); // Temperature

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &message);

	mavcam_cb_send_raw_bytes(buf,len);
}

static void loop(void) {
    uint8_t recv_buf[2048];
    
    while (true) {
        uint32_t now = get_time_boot_ms();
        uint64_t utc = get_current_utc_us();

        if(enable_temperature_wifilink2) {
            send_temp_to_gcs();
        }
        
        mavcam_spin(now, utc);

        // Read incoming UDP bytes from
        ssize_t bytes_recv = recv(sock_fd, &recv_buf[0], sizeof(recv_buf), 0);

        if (bytes_recv > 0) {
            mavcam_update(recv_buf,bytes_recv,now,utc);
        }
    }
}

int main(int argc, char *argv[]) {

    curl_global_init(CURL_GLOBAL_ALL);

    mavcam_init(0, MAV_COMP_ID_CAMERA);

    init_storages();
    init_rest_api();

    int status = parse_cli(argc,argv);
    if(status != EXIT_SUCCESS)
        goto prog_end;

    status = init_network();
    if(status != EXIT_SUCCESS)
        goto prog_end;
  
    LOG_INFO("Network initialized. Entering main loop...");

    loop();

prog_end:
    
    free_network();
    free_rest_api();
    
    curl_global_cleanup();
    
    return status;
}

//---------------------------------------------------------------------------//
//------------- MAVCAM HARDWARE LAYER IMPLEMENTATION ------------------------//
//---------------------------------------------------------------------------//

void mavcam_cb_send_raw_bytes(const uint8_t *buffer, uint16_t length) {
    // 1. Validate the active state of the initialization network socket descriptor
    if (sock_fd < 0) {
        LOG_ERROR("Network output failure: UDP socket is not initialized or closed.");
        return;
    }

    // 2. Prevent boundary processing issues with empty payloads or null references
    if (buffer == NULL || length == 0) {
        LOG_WARN("Network output bypassed: Outbound buffer is empty or payload length is zero.");
        return;
    }

    // 3. Flush the raw binary MAVLink frame via the OS networking layer socket interface
    ssize_t bytes_sent = sendto(sock_fd, 
                                buffer, 
                                (size_t)length, 
                                0, 
                                (struct sockaddr *)&router_addr, 
                                sizeof(router_addr));

    // 4. Evaluate transactional integrity status parameters of the sent packet stream
    if (bytes_sent < 0) {
        // Parse the internal Linux standard kernel error string if interface queues overflow
        LOG_ERROR("UDP transmission failed. System error: %s (errno: %d)", strerror(errno), errno);
    } else if (bytes_sent != (ssize_t)length) {
        // Track exceptional partial write discrepancies inside the network driver layer
        LOG_WARN("Network transmission frame size mismatch. Expected: %u bytes, sent: %zd bytes.", 
                 length, bytes_sent);
    }
}

bool mavcam_cb_query_storage_metrics(uint8_t storage_id, float *used_mib, float *available_mib, float *total_mib) {
    struct statvfs vfs;

    storage_t *p_storage = get_storage(storage_id);

    if(!p_storage) {
        LOG_ERROR("Can't query storage metrics - Storage ID %d is not found", storage_id);
        return false;
    }

    if (statvfs(p_storage->dcim, &vfs) == 0) {
        double total_bytes = (double)vfs.f_blocks * (double)vfs.f_frsize;
        double available_bytes = (double)vfs.f_bavail * (double)vfs.f_frsize;
        double used_bytes = total_bytes - available_bytes;

        if(used_mib)
            *used_mib = (float)(used_bytes) / (1024.f * 1024.f);
        if(available_mib)
            *available_mib = (float)(available_bytes) / (1024.f * 1024.f);
        if(total_mib)
            *total_mib = (float)(total_bytes) / (1024.f * 1024.f);
    } else {
        LOG_ERROR("Failed to query filesystem metrics for path: %s", p_storage->dcim);
        return false;
    }

    return true;
}

int32_t mavcam_cb_video_query_status_is_recording(uint8_t stream_id) {
    
    switch(venc) {
        case VENC_WAYBEAM:
            return waybeam_query_status_is_recording(stream_id);
        default:
            break;
    }

    return -1;
}

bool mavcam_cb_save_image_metadata(const camera_image_meta_t *meta) {
    char meta_path[512];

    if(!meta || strlen(meta->file_url) == 0) {
        LOG_ERROR("Failed to create metadata file - invalid meta->file_url");
        return false;
    }
    
    // Find the last occurrence of the dot character and the slash character
    const char *dot = strrchr(meta->file_url, '.');
    const char *slash = strrchr(meta->file_url, '/');
    
    // The dot is valid only if it exists and comes after the last slash component
    if (dot && (!slash || dot > slash)) {
        // Calculate the base path length excluding the extension and the dot
        size_t base_len = dot - meta->file_url;
        
        // Prevent buffer overflow by limiting the maximum copy size
        if (base_len >= sizeof(meta_path) - 6) {
            base_len = sizeof(meta_path) - 6;
        }
        
        // Copy the base path up to the dot and append the clean .meta extension
        snprintf(meta_path, sizeof(meta_path), "%.*s.meta", (int)base_len, meta->file_url);
    } else {
        // Fallback constraint if the filename does not contain any extension
        snprintf(meta_path, sizeof(meta_path), "%s.meta", meta->file_url);
    }

    // Open file pointer in write-binary mode
    FILE *fp = fopen(meta_path, "wb");
    if (!fp) {
        LOG_ERROR("Failed to create metadata file on disk: %s", meta_path);
        return false;
    }

    // Serialize and write the clean memory structure directly onto the block storage
    size_t written = fwrite(meta, sizeof(camera_image_meta_t), 1, fp);
    fclose(fp);

    // Verify if the block was written fully to prevent corrupted or partial filesystem entries
    if (written != 1) {
        LOG_ERROR("Failed to write the full metadata block to file: %s", meta_path);
        remove(meta_path); // Clean up partial artifacts
        return false;
    }

    LOG_DEBUG("Saved image metadata block to: %s", meta_path);
    return true;
}

bool mavcam_cb_load_image_metadata_by_index(int32_t index, camera_image_meta_t *meta_out) {
    char meta_path[512];
    
    storage_t *p_storage = get_storage(current_storage_id);
    
    if(!p_storage) {
        LOG_ERROR("Failed to open image metadata, Storage ID %d is not found", current_storage_id);
        return false;
    }

    DIR *dir = opendir(p_storage->dcim);
    if (!dir) {
        LOG_ERROR("Failed to open storage directory for metadata lookup");
        return false;
    }

    struct dirent *entry;
    char expected_prefix[32];
    // Create the unique index pattern based on the requested signed integer 
    snprintf(expected_prefix, sizeof(expected_prefix), CAMERA_IMAGE_NAME_PREFIX_FMT, index);
    bool found = false;

    while ((entry = readdir(dir)) != NULL) {
        // Match files starting with our index prefix and ending strictly with the .meta extension
        if (strncmp(entry->d_name, expected_prefix, strlen(expected_prefix)) == 0 &&
            strstr(entry->d_name, ".meta") != NULL) {
            
            snprintf(meta_path, sizeof(meta_path), "%s/%s", p_storage->dcim, entry->d_name);
            FILE *fp = fopen(meta_path, "rb");
            if (fp) {
                // Deserialize the clean memory structure package directly from disk
                if (fread(meta_out, sizeof(camera_image_meta_t), 1, fp) == 1) {
                    found = true;
                }
                fclose(fp);
            }
            break;
        }
    }
    
    closedir(dir);

    return found;
}

bool mavcam_cb_video_start_capture(uint8_t stream_id) {
    if (!rest_api.base_url || !rest_api.video_start) {
        LOG_ERROR("Failed to start video capture: REST API configuration or 'video_start' endpoint is missing.");
        return false;
    }

    bool success = send_rest_request(rest_api.video_start);

    if (success) {
        LOG_INFO("Video capture pipeline successfully initiated via REST API for Stream ID: %u", stream_id);
    } else {
        LOG_ERROR("Hardware interface failed to accept video start command for Stream ID: %u", stream_id);
    }

    return success;
}

bool mavcam_cb_video_stop_capture(uint8_t stream_id) {
    if (!rest_api.base_url || !rest_api.video_stop) {
        LOG_ERROR("Failed to stop video capture: REST API configuration or 'video_stop' endpoint is missing.");
        return false;
    }

    bool success = send_rest_request(rest_api.video_stop);

    if (success) {
        LOG_INFO("Video capture pipeline successfully terminated via REST API for Stream ID: %u", stream_id);
    } else {
        LOG_ERROR("Hardware interface failed to accept video stop command for Stream ID: %u", stream_id);
    }

    return success;    
}

bool mavcam_cb_video_start_stream(uint8_t stream_id) {
    (void)stream_id;
    return true;
}

bool mavcam_cb_video_stop_stream(uint8_t stream_id) {
    (void)stream_id;
    return true;
}


int32_t mavcam_cb_image_capture(uint32_t sequence, char *file_url, size_t file_url_size) {
    if (!rest_api.base_url || !rest_api.photo_capture) {
        LOG_ERROR("Failed to capture image: REST API configuration or 'photo_capture' endpoint is missing.");
        return -1;
    }
    storage_t *p_storage = get_storage(current_storage_id);
    if (!p_storage || strlen(p_storage->dcim) == 0) {
        LOG_ERROR("Failed to capture image: Target storage ID %d is not configured or path is empty.", current_storage_id);
        return -1;
    }

    int32_t current_idx = p_storage->last_image_idx + 1;

    if(sequence > 0) {
        current_idx = sequence;
    }

    // 4. Construct the absolute destination filename on the Linux filesystem structure
    char local_path[512];
    char filename_buf[64];
    
    // Format the clean index into standard image naming framework (e.g., "IMG_000042.jpg")
    snprintf(filename_buf, sizeof(filename_buf), CAMERA_IMAGE_NAME_WRITE_FMT, current_idx);
    // Combine directory root mount path with target filename
    snprintf(local_path, sizeof(local_path), "%s/%s", p_storage->dcim, filename_buf);

    bool capture_success = download_and_save_photo(rest_api.photo_capture, local_path);

    if (!capture_success) {
        LOG_ERROR("Hardware snapshot capture transaction failed to execute or write to disk.");
        return -1;
    }

    if (file_url != NULL) {
        strncpy(file_url, local_path, file_url_size - 1);
    }

    p_storage->image_count++;
    p_storage->last_image_idx = current_idx;

    LOG_DEBUG("Snapshot captured successfully! File index: %d written to: %s", current_idx, local_path);
    
    return current_idx;
}

bool mavcam_cb_set_camera_mode(uint8_t camera_mode) {
    if (!rest_api.base_url) {
        LOG_ERROR("Failed to transition camera mode: REST API base URL configuration is missing.");
        return false;
    }

    const char *target_endpoint = NULL;

    switch (camera_mode) {
        case CAMERA_MODE_IMAGE: // Mode 0
            target_endpoint = rest_api.set_photo_mode;
            break;

        case CAMERA_MODE_VIDEO: // Mode 1
            target_endpoint = rest_api.set_video_mode;
            break;

        default:
            LOG_WARN("Transition rejected: Requested mode classification index %u is unknown or unsupported.", camera_mode);
            return false;
    }

    if (!target_endpoint || strlen(target_endpoint) == 0) {
        // REST API to change mode is not mandatory
        return true;
    }

    bool success = send_rest_request(target_endpoint);

    if (success) {
        LOG_INFO("Camera hardware mode successfully synchronized to destination state: %u", camera_mode);
    } else {
        LOG_ERROR("Hardware interface failed to accept mode transition command to state: %u", camera_mode);
    }

    return success;
}

bool mavcam_cb_reset_camera_settings(uint8_t reset_mode) {
    (void)reset_mode;
    return true;
}

static void remove_dir_content(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        LOG_WARN("Could not open directory for cleaning: %s", path);
        return;
    }

    struct dirent *entry;
    char full_path[512];

    while ((entry = readdir(dir)) != NULL) {

        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);

        if (entry->d_type == DT_DIR) {
            remove_dir_content(full_path);
            if (rmdir(full_path) != 0) {
                LOG_ERROR("Failed to remove subdirectory %s: %s", full_path, strerror(errno));
            }
        } else {
            if (unlink(full_path) != 0) {
                LOG_ERROR("Failed to delete file %s: %s", full_path, strerror(errno));
            }
        }
    }

    closedir(dir);
}

bool mavcam_cb_storage_format(uint8_t storage_id) {
    storage_t *p_storage = get_storage(storage_id);
    if (!p_storage || strlen(p_storage->dcim) == 0) {
        LOG_ERROR("Failed to format storage: Storage ID %d is not configured or path is empty.", storage_id);
        return false;
    }

    LOG_INFO("Local storage format initiated. Cleaning directory: %s", p_storage->dcim);

    remove_dir_content(p_storage->dcim);

    p_storage->image_count = 0;
    p_storage->last_image_idx = 0;

    LOG_INFO("Storage ID %d format complete. Content erased, counters reset.", storage_id);
    return true;
}