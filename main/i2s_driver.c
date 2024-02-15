#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "i2s_driver.h"

static	i2s_chan_handle_t	rx_chan;	// I2S rx channel handler

//uint16_t *r_buf;
uint16_t r_buf[BUFF_SIZE_I2S];
size_t r_bytes=0;
int16_t *chan_1=NULL;
int16_t *chan_2=NULL;

void i2s_configure_channel(){

	//r_buf=(uint16_t *)calloc(1,BUFF_SIZE_I2S);

	//assert(r_buf);
	//i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	i2s_chan_config_t chan_cfg ={
		.id=I2S_NUM_AUTO,
		.role = I2S_ROLE_MASTER,//I2S_ROLE_SLAVE,//I2S_ROLE_MASTER,//
		.dma_desc_num=4,
		.dma_frame_num = 1024,
    	.auto_clear = false
	};
	ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg,NULL, &rx_chan));
 
    i2s_std_config_t std_cfg = {
    	.clk_cfg  = {
		    .sample_rate_hz = 44100,
		    .clk_src = I2S_CLK_SRC_DEFAULT,//I2S_CLK_SRC_DEFAULT,I2S_CLK_SRC_PLL_160M,I2S_CLK_SRC_APLL
		    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    	},
    	.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    	/*.slot_cfg = {
			.data_bit_width=I2S_DATA_BIT_WIDTH_32BIT,
			.slot_bit_width=I2S_SLOT_BIT_WIDTH_AUTO, //I2S_SLOT_BIT_WIDTH_8BIT,I2S_SLOT_BIT_WIDTH_16BIT,I2S_SLOT_BIT_WIDTH_24BIT,I2S_SLOT_BIT_WIDTH_32BIT
			.slot_mode=I2S_SLOT_MODE_STEREO,
			.slot_mask=I2S_STD_SLOT_RIGHT,//I2S_STD_SLOT_RIGHT,I2S_STD_SLOT_BOTH,I2S_STD_SLOT_LEFT
			.ws_width=I2S_DATA_BIT_WIDTH_32BIT,
			.ws_pol=false,
			.bit_shift=true,//Configurado para habilitar el cambio de bits en el modo Philips
			.msb_right = true,//Configurado para colocar los datos del canal correcto en el MSB en el FIFO
    	},*/
    	.gpio_cfg = {
        	.mclk = GPIO_NUM_0,
        	.bclk = GPIO_NUM_19,
        	.ws   = GPIO_NUM_22,
        	.dout = -1,
        	.din  = GPIO_NUM_17, // In duplex mode, bind output and input to a same gpio can loopback internally
        	.invert_flags = {
            	.mclk_inv = false,
            	.bclk_inv = false,
            	.ws_inv   = false,
        	},
    	},
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
}

void i2s_read(){



	i2s_channel_enable(rx_chan);
	if (i2s_channel_read(rx_chan, (void*)r_buf, sizeof(r_buf), &r_bytes, 1000) == ESP_OK) {

        for (int16_t i = 0; i < r_bytes/4; i+=2) {
			if (r_buf[i*2+1] & 0x8000) { // Comprueba el bit más significativo para determinar si es negativo
				chan_1[i/2]=(int16_t)(r_buf[i*2+1] | 0xFFFF0000);
			} else {
				chan_1[i/2]=(int16_t)r_buf[i*2+1];
			}
			if (r_buf[i*2+3] & 0x8000) { // Comprueba el bit más significativo para determinar si es negativo
				chan_2[i/2]=(int16_t)(r_buf[i*2+3] | 0xFFFF0000);
			} else {
				chan_2[i/2]=(int16_t)r_buf[i*2+3];
			}
			printf("%d\n",chan_1[i/2]);
     	}
    }else {
        printf("Read Task: i2s read failed\n");
    }
	i2s_channel_disable(rx_chan);
	//free(r_buf);
}

uint16_t* i2s_raw(){

	i2s_channel_enable(rx_chan);
	if (i2s_channel_read(rx_chan,r_buf, sizeof(r_buf), &r_bytes, portMAX_DELAY)!= ESP_OK){
		printf("Read Task: i2s read failed\n");
	}
	i2s_channel_disable(rx_chan);
	return r_buf;
}