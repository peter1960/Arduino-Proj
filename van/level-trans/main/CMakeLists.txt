idf_component_register(
    SRCS 
        "main.cpp"
        "mpu6050.cpp"
        "level_filter.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)
