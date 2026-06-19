#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "wifi_manager.h"
#include "web_dashboard.h"
#include "obstacle_avoidance.h"

void app_main(void)
{
    printf("Tugas Besar Sistem Kendali starting...\n");

    motor_control_init();

    wifi_manager_init();

    web_dashboard_start();

    obstacle_avoidance_start();

    printf("Ready! Open http://project.local in browser\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
