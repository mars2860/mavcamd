#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdbool.h>
#include <string.h>

char *camera_config_path = NULL;
char *router_arg = NULL;

char router_ip[64] = {0};
int router_port = 0;

static void print_usage(const char *prog_name) {
    fprintf(stderr, "Mavlink Camera Protocol Daemon for OpenIPC\n\n");
    fprintf(stderr, "Usage: %s -a <ip:port> -c <config_file> [options]\n\n", prog_name);
    fprintf(stderr, "Required options:\n");
    fprintf(stderr, "  -a, --address <ip:port>   IP address and UDP port for mavlink traffic\n");
    fprintf(stderr, "  -c, --config <file>       Path to Mavlink Camera configuration JSON file\n");
    fprintf(stderr, "Other options:\n");
    fprintf(stderr, "  -h, --help                Show this help message\n");
}

int main(int argc, char *argv[]) {
    static struct option long_options[] = {
        {"address", required_argument, 0, 'a'},
        {"config",  required_argument, 0, 'c'},
        {"help",    no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int opt;

    while ((opt = getopt_long(argc, argv, "a:c:h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'a':
                router_arg = optarg;
                break;
            case 'c':
                camera_config_path = optarg;
                break;
            case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    if (router_arg == NULL || camera_config_path == NULL) {
        fprintf(stderr, "[mavcamd] Error: Both --address (-a) and --config (-c) are required parameters!\n\n");
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    char *colon = strchr(router_arg, ':');
    if (!colon) {
        fprintf(stderr, "[mavcamd] Error: Invalid address format '%s'. Expected format is IP:PORT (e.g. 127.0.0.1:14553).\n", router_arg);
        return EXIT_FAILURE;
    }

    *colon = '\0';
    strncpy(router_ip, router_arg, sizeof(router_ip) - 1);
    router_port = atoi(colon + 1);
    *colon = ':';

    if (router_port <= 0 || router_port > 65535) {
        fprintf(stderr, "[mavcamd] Error: Invalid port number '%d'. Must be between 1 and 65535.\n", router_port);
        return EXIT_FAILURE;
    }

    printf("[mavcamd] Target router address parsed: %s:%d\n", router_ip, router_port);
    printf("[mavcamd] Camera config file path: %s\n", camera_config_path);

    // TODO: Следующим шагом здесь будет вызов load_camera_config(camera_config_path)

    return EXIT_SUCCESS;
}
