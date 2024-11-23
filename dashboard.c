#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define SERVER_IP "172.20.10.11"  // Set the server's IP address
#define TCP_PORT 4242
#define BUF_SIZE 2048

// Define the structure expected from the server
typedef struct {
    float forward_spd;
    float backward_spd;
    float turn_right_spd;
    float turn_left_spd;
    bool stop;
    bool auto_mode;
} dirCommands;

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *client_pcb;
    bool connected;
    uint8_t buffer_recv[BUF_SIZE];
} TCP_CLIENT_T;

static TCP_CLIENT_T *tcp_client_init(void) {
    TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        printf("Failed to allocate memory for TCP client state.\n");
        return NULL;
    }
    return state;
}

static err_t tcp_client_close(TCP_CLIENT_T *state) {
    if (state->client_pcb) {
        tcp_arg(state->client_pcb, NULL);
        tcp_close(state->client_pcb);
        state->client_pcb = NULL;
    }
    free(state);
    return ERR_OK;
}

static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (!p) {  // Server has closed the connection
        printf("Connection closed by the server.\n");
        tcp_client_close(state);
        return ERR_OK;
    }

    if (p->tot_len > 0) {
        printf("Data received from server (%d bytes).\n", p->tot_len);

        // Copy the received data into the local buffer
        size_t len = pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        state->buffer_recv[len] = '\0'; // Null-terminate the received buffer for safety

        // Check if the data matches the expected size of the structure
        if (len == sizeof(dirCommands)) {
            dirCommands *received_cmd = (dirCommands *)state->buffer_recv;

            printf("Receiving Direction Commands...\n");
            printf("[Accelerometer]  Forward Speed: %.2f\n", received_cmd->forward_spd);
            printf("[Accelerometer]  Backward Speed: %.2f\n", received_cmd->backward_spd);
            printf("[Accelerometer]  Turn Right Speed: %.2f\n", received_cmd->turn_right_spd);
            printf("[Accelerometer]  Turn Left Speed: %.2f\n", received_cmd->turn_left_spd);
            printf("[Accelerometer] Stop: %s\n", received_cmd->stop ? "True" : "False");
            printf("[Accelerometer]  Auto Mode: %s\n", received_cmd->auto_mode ? "True" : "False");
        } else {
            printf("Received unexpected data size: %d bytes\n", len);
            printf("Data: %s\n", state->buffer_recv);
        }

        // Indicate that the data has been received
        tcp_recved(tpcb, p->tot_len);
    }

    pbuf_free(p);  // Free the received buffer
    return ERR_OK; // Continue receiving
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (err != ERR_OK) {
        printf("Connection error: %d\n", err);
        tcp_client_close(state);
        return err;
    }

    printf("Successfully connected to the server.\n");
    state->connected = true;
    tcp_recv(tpcb, tcp_client_recv); // Set the receive callback
    return ERR_OK;
}

static bool tcp_client_connect(TCP_CLIENT_T *state) {
    state->client_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!state->client_pcb) {
        printf("Failed to create new TCP control block.\n");
        return false;
    }

    ip4_addr_t server_addr;
    if (!ip4addr_aton(SERVER_IP, &server_addr)) {
        printf("Invalid server IP address format.\n");
        return false;
    }

    err_t err = tcp_connect(state->client_pcb, &server_addr, TCP_PORT, tcp_client_connected);
    if (err != ERR_OK) {
        printf("Failed to initiate TCP connection: %d\n", err);
        tcp_client_close(state);
        return false;
    }

    tcp_arg(state->client_pcb, state);
    return true;
}

void run_tcp_client_test(void) {
    TCP_CLIENT_T *state = tcp_client_init();
    if (!state) {
        return;
    }

    if (!tcp_client_connect(state)) {
        tcp_client_close(state);
        return;
    }

    while (true) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(500));
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("WiFi initialization failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connection failed\n");
        return 1;
    }

    run_tcp_client_test();
    cyw43_arch_deinit();
    return 0;
}
