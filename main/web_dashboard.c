#include "web_dashboard.h"
#include "shared_data.h"
#include "motor_control.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_http_server.h"
#include "cJSON.h"

extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[] asm("_binary_dashboard_html_end");

static httpd_handle_t server = NULL;

/* ---- shared state for websocket push ---- */
#define WS_CLIENTS_MAX 4
static int ws_clients[WS_CLIENTS_MAX];
static int ws_client_count = 0;

static void ws_send_json(const char *json)
{
    httpd_ws_frame_t ws_pkt = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json,
        .len = strlen(json),
    };
    for (int i = 0; i < ws_client_count; i++) {
        httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
    }
}

/* ---- REST handlers (unchanged) ---- */

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)dashboard_html_start,
                   dashboard_html_end - dashboard_html_start);
    return ESP_OK;
}

static cJSON *build_status_json(void)
{
    cJSON *root = cJSON_CreateObject();

    cJSON *sensors = cJSON_CreateObject();
    cJSON_AddNumberToObject(sensors, "front", (double)sensor_front_cm);
    cJSON_AddNumberToObject(sensors, "left", (double)sensor_front_left_cm);
    cJSON_AddNumberToObject(sensors, "right", (double)sensor_front_right_cm);
    cJSON_AddNumberToObject(sensors, "back", (double)sensor_back_cm);
    cJSON_AddItemToObject(root, "sensors", sensors);

    cJSON *pid = cJSON_CreateObject();
    cJSON_AddNumberToObject(pid, "kp", speed_pid.kp);
    cJSON_AddNumberToObject(pid, "ki", speed_pid.ki);
    cJSON_AddNumberToObject(pid, "kd", speed_pid.kd);
    cJSON_AddNumberToObject(pid, "error", speed_pid.prev_error);
    cJSON_AddNumberToObject(pid, "output", pid_output_display);
    cJSON_AddNumberToObject(pid, "setpoint", speed_setpoint);
    cJSON_AddItemToObject(root, "pid", pid);

    cJSON_AddBoolToObject(root, "auto_mode", auto_mode);
    cJSON_AddNumberToObject(root, "drive_mode", drive_mode);

    cJSON *graph = cJSON_CreateObject();
    cJSON *err_arr = cJSON_CreateArray();
    cJSON *out_arr = cJSON_CreateArray();
    cJSON *sp_arr = cJSON_CreateArray();
    int count = GRAPH_SIZE;
    if (graph_sample_count < count) count = graph_sample_count;
    for (int i = 0; i < count; i++) {
        int idx = (graph_index - count + i + GRAPH_SIZE) % GRAPH_SIZE;
        cJSON_AddItemToArray(err_arr, cJSON_CreateNumber(graph_error[idx]));
        cJSON_AddItemToArray(out_arr, cJSON_CreateNumber(graph_output[idx]));
        cJSON_AddItemToArray(sp_arr, cJSON_CreateNumber(graph_setpoint[idx]));
    }
    cJSON_AddItemToObject(graph, "error", err_arr);
    cJSON_AddItemToObject(graph, "output", out_arr);
    cJSON_AddItemToObject(graph, "setpoint", sp_arr);
    cJSON_AddItemToObject(root, "graph", graph);

    return root;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    cJSON *root = build_status_json();
    const char *json = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

static void execute_command(const char *cmd, int speed)
{
    manual_override = 1;
    override_count = 0;

    if (strcmp(cmd, "forward") == 0) {
        motor_set_speed(speed, speed, speed, speed);
    } else if (strcmp(cmd, "backward") == 0) {
        motor_set_speed(-speed, -speed, -speed, -speed);
    } else if (strcmp(cmd, "left") == 0) {
        motor_set_speed(speed, -speed, speed, -speed);
    } else if (strcmp(cmd, "right") == 0) {
        motor_set_speed(-speed, speed, -speed, speed);
    } else if (strcmp(cmd, "stop") == 0) {
        motor_all_stop();
        manual_override = 0;
        override_count = 0;
    }
}

static esp_err_t control_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON");
        return ESP_FAIL;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    cJSON *speed_item = cJSON_GetObjectItem(root, "speed");

    if (cmd_item && cmd_item->valuestring) {
        int speed = 70;
        if (speed_item) speed = (int)speed_item->valuedouble;
        execute_command(cmd_item->valuestring, speed);
    }

    cJSON_Delete(root);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "OK");
    const char *json = cJSON_Print(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free((void *)json);
    cJSON_Delete(resp);
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON");
        return ESP_FAIL;
    }

    cJSON *mode = cJSON_GetObjectItem(root, "mode");
    if (mode && mode->valuestring) {
        if (strcmp(mode->valuestring, "auto") == 0) {
            auto_mode = 1;
            manual_override = 0;
            override_count = 0;
            printf("Mode: AUTO\n");
        } else if (strcmp(mode->valuestring, "stop") == 0) {
            auto_mode = 0;
            manual_override = 0;
            motor_all_stop();
            motor_fr_speed = 0;
            motor_fl_speed = 0;
            motor_br_speed = 0;
            motor_bl_speed = 0;
            printf("Mode: STOP\n");
        } else if (strcmp(mode->valuestring, "cruise") == 0) {
            drive_mode = DRIVE_MODE_CRUISE;
            auto_mode = 1;
            manual_override = 0;
            override_count = 0;
            printf("Drive: CRUISE\n");
        } else if (strcmp(mode->valuestring, "maintain") == 0) {
            drive_mode = DRIVE_MODE_MAINTAIN;
            auto_mode = 1;
            manual_override = 0;
            override_count = 0;
            printf("Drive: MAINTAIN\n");
        }
    }

    cJSON_Delete(root);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "OK");
    const char *json = cJSON_Print(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free((void *)json);
    cJSON_Delete(resp);
    return ESP_OK;
}

static esp_err_t pid_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad JSON");
        return ESP_FAIL;
    }

    cJSON *kp_item = cJSON_GetObjectItem(root, "kp");
    cJSON *ki_item = cJSON_GetObjectItem(root, "ki");
    cJSON *kd_item = cJSON_GetObjectItem(root, "kd");
    cJSON *sp_item = cJSON_GetObjectItem(root, "setpoint");

    if (kp_item) speed_pid.kp = (float)kp_item->valuedouble;
    if (ki_item) speed_pid.ki = (float)ki_item->valuedouble;
    if (kd_item) speed_pid.kd = (float)kd_item->valuedouble;
    if (sp_item) speed_setpoint = (float)sp_item->valuedouble;
    pid_ctrl_reset(&speed_pid);
    pid_ctrl_save_to_nvs(&speed_pid, speed_setpoint);
    printf("PID: Kp=%.2f Ki=%.3f Kd=%.3f Set=%.1f\n", speed_pid.kp, speed_pid.ki, speed_pid.kd, speed_setpoint);

    cJSON_Delete(root);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "PID set");
    const char *json = cJSON_Print(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free((void *)json);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* ---- WebSocket handler ---- */

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int sockfd = httpd_req_to_sockfd(req);
        if (ws_client_count < WS_CLIENTS_MAX) {
            ws_clients[ws_client_count++] = sockfd;
            printf("WebSocket client connected (fd=%d, total=%d)\n", sockfd, ws_client_count);
        }
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        int sockfd = httpd_req_to_sockfd(req);
        for (int i = 0; i < ws_client_count; i++) {
            if (ws_clients[i] == sockfd) {
                ws_clients[i] = ws_clients[--ws_client_count];
                printf("WebSocket client disconnected (fd=%d, total=%d)\n", sockfd, ws_client_count);
                break;
            }
        }
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
        ws_pkt.type = HTTPD_WS_TYPE_PONG;
        httpd_ws_send_frame(req, &ws_pkt);
        return ESP_OK;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        int sockfd = httpd_req_to_sockfd(req);
        for (int i = 0; i < ws_client_count; i++) {
            if (ws_clients[i] == sockfd) {
                ws_clients[i] = ws_clients[--ws_client_count];
                printf("WebSocket client disconnected (fd=%d, total=%d)\n", sockfd, ws_client_count);
                break;
            }
        }
        return ESP_OK;
    }

    return ESP_OK;
}

/* ---- WebSocket push task ---- */

static void ws_push_task(void *arg)
{
    esp_task_wdt_add(NULL);
    char buf[5120];
    while (1) {
        if (ws_client_count > 0) {
            cJSON *root = build_status_json();
            cJSON_PrintPreallocated(root, buf, sizeof(buf), 0);
            ws_send_json(buf);
            cJSON_Delete(root);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_task_wdt_reset();
    }
}

/* ---- Public: start server ---- */

void web_dashboard_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = {
            .uri = "/api/status",
            .method = HTTP_GET,
            .handler = status_handler
        };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t control_uri = {
            .uri = "/api/control",
            .method = HTTP_POST,
            .handler = control_handler
        };
        httpd_register_uri_handler(server, &control_uri);

        httpd_uri_t mode_uri = {
            .uri = "/api/mode",
            .method = HTTP_POST,
            .handler = mode_handler
        };
        httpd_register_uri_handler(server, &mode_uri);

        httpd_uri_t pid_uri = {
            .uri = "/api/pid",
            .method = HTTP_POST,
            .handler = pid_handler
        };
        httpd_register_uri_handler(server, &pid_uri);

        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .is_websocket = true,
        };
        httpd_register_uri_handler(server, &ws_uri);

        xTaskCreate(ws_push_task, "ws_push", 8192, NULL, 3, NULL);

        printf("Dashboard on port 80 (WebSocket /ws)\n");
    }
}