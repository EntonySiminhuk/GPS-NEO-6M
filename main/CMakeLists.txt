idf_component_register(
    SRCS main.c oled.c
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_driver_i2c
)

# Trata qualquer aviso do nosso codigo como erro
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
