/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp32_i2c_rw.h"
#include "mpu6050.h"

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    i2c_master_init();
    mpu6050_init();

    printf("Testing device connections..\n");
    printf(mpu6050_test_connection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    mpu6050_acceleration_t aData;
    mpu6050_rotation_t gData;

    while(1){
        mpu6050_get_acceleration(&aData);
        mpu6050_get_rotation(&gData);

        printf("a/g:\t");
        printf("%d", aData.accel_x);
        printf("\t");
        printf("%d", aData.accel_y);
        printf("\t");
        printf("%d", aData.accel_z);
        printf("\t");
        printf("%d", gData.gyro_x);
        printf("\t");
        printf("%d", gData.gyro_y);
        printf("\t");
        printf("%d", gData.gyro_z);
        printf("\n");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
