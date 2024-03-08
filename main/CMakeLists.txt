idf_component_register(SRCS "read-and-show.c" "ili9340.c" "fontx.c"
                    INCLUDE_DIRS ".")
                    
set(COMPONENT_REQUIRES
    esp_https_server
)
