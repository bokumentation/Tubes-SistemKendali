idf_component_register(
    SRCS "main.c"
         "motor_control.c"
         "ultrasonic.c"
         "pid_controller.c"
         "obstacle_avoidance.c"
         "wifi_manager.c"
         "web_dashboard.c"
    INCLUDE_DIRS "." "include"
    PRIV_REQUIRES freertos driver esp_timer esp_wifi esp_event nvs_flash esp_http_server esp_driver_mcpwm esp_driver_gpio esp_driver_rmt
    EMBED_TXTFILES html/dashboard.html
)
