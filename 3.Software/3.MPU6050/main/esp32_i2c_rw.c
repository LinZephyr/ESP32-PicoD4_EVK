/**
 * @file esp32_i2c_rw.c
 * 
 * @author
 * Gabriel Boni Vicari (133192@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 *
 * @copyright 2012 Jeff Rowberg
 *
 * @brief I2C Read/Write functions for ESP32 ESP-IDF.
 */

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32_i2c_rw.h"

#define I2C_NUM (I2C_NUM_0)

#define CONFIG_I2C_MASTER_SCL 33
#define CONFIG_I2C_MASTER_SDA 32
#define CONFIG_I2C_MASTER_PORT_NUM I2C_NUM
#define CONFIG_I2C_MASTER_FREQUENCY 100000


#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void select_register(uint8_t device_address, uint8_t register_address)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, register_address, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

int8_t esp32_i2c_read_bytes
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t size,
	uint8_t* data
)
{
	select_register(device_address, register_address);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, 1);

	if (size > 1)
		i2c_master_read(cmd, data, size - 1, 0);

	i2c_master_read_byte(cmd, data + size - 1, 1);

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return (size);
}

int8_t esp32_i2c_read_byte
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t* data
)
{
	return (esp32_i2c_read_bytes(device_address, register_address, 1, data));
}

int8_t esp32_i2c_read_bits
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_start,
	uint8_t size,
	uint8_t* data
)
{
	uint8_t bit;
	uint8_t count;

	if ((count = esp32_i2c_read_byte(device_address, register_address, &bit))) {
		uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);

		bit &= mask;
		bit >>= (bit_start - size + 1);
		*data = bit;
	}

	return (count);
}

int8_t esp32_i2c_read_bit
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t* data
)
{
	uint8_t bit;
	uint8_t count = esp32_i2c_read_byte(device_address, register_address, &bit);

	*data = bit & (1 << bit_number);

	return (count);
}

bool esp32_i2c_write_bytes
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t size,
	uint8_t* data
)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, register_address, 1);
	i2c_master_write(cmd, data, size - 1, 0);
	i2c_master_write_byte(cmd, data [size - 1], 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return (true);
}

bool esp32_i2c_write_byte
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t data
)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, register_address, 1);
	i2c_master_write_byte(cmd, data, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return (true);
}

bool esp32_i2c_write_bits
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_start,
	uint8_t size,
	uint8_t data
)
{
	uint8_t bit = 0;

	if (esp32_i2c_read_byte(device_address, register_address, &bit) != 0) {
		uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);
		data <<= (bit_start - size + 1);
		data &= mask;
		bit &= ~(mask);
		bit |= data;
		return (esp32_i2c_write_byte(device_address, register_address, bit));
	}
	else
		return (false);
}

bool esp32_i2c_write_bit
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t data
)
{
	uint8_t bit;

	esp32_i2c_read_byte(device_address, register_address, &bit);

	if (data != 0)
		bit = (bit | (1 << bit_number));
	else
		bit = (bit & ~(1 << bit_number));

	return (esp32_i2c_write_byte(device_address, register_address, bit));
}

int8_t esp32_i2c_write_word
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t data
)
{
	uint8_t data_1[] = {(uint8_t) (data >> 8), (uint8_t) (data & 0xFF)};

	esp32_i2c_write_bytes(device_address, register_address, 2, data_1);

	return (1);
}
