#ifndef MAIN_I2S_DRIVER_H_
#define MAIN_I2S_DRIVER_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2s_std.h"

#define	BUFF_SIZE_I2S	2048

extern int16_t *chan_1;
extern int16_t *chan_2;
/**
 * @brief      Configura el canal I2S para leer y escribir
 */
void i2s_configure_channel();
/**
 * @brief      Debe tomar al menos 20 muestras de la señal leida
 */
void i2s_read();
/**
 * @brief      Lee la conversion ADC
 *
 * @return     Devuelve el buffer sin ordenar
 */
uint16_t* i2s_raw();



#endif /* MAIN_I2S_DRIVER_H_ */