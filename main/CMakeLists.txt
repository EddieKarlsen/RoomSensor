idf_component_register(
    SRCS
        app_main.c
        src/communication.c
        src/pid.c
    
    INCLUDE_DIRS 
        .
        inc

    PRIV_REQUIRES 
        driver
        esp_timer
        json 
)