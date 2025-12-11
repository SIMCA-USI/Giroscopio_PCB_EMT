#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "MPU6050"

// Pines I2C (según tu esquemático: SDA=21, SCL=22)
#define I2C_MASTER_SDA_IO           21      // GPIO para SDA
#define I2C_MASTER_SCL_IO           22      // GPIO para SCL
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Dirección I2C de la MPU6050
#define MPU6050_ADDR        0x68

// Registros
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SENS_2G      16384.0f   // LSB/g
#define GYRO_SENS_250DPS   131.0f     // LSB/(°/s)

static esp_err_t i2c_master_init_debug(void)
{
    ESP_LOGI(TAG, "Configurando I2C en SDA=%d, SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_param_config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C inicializado correctamente");
    return ESP_OK;
}

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_init(void)
{
    ESP_LOGI(TAG, "Despertando MPU6050 (quitando SLEEP)");
    esp_err_t ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo PWR_MGMT_1: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t who_am_i = 0;
    ESP_LOGI(TAG, "Leyendo WHO_AM_I...");
    ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who_am_i);
    if (who_am_i != 0x68) {
        ESP_LOGW(TAG, "WHO_AM_I inesperado (esperado 0x68)");
    } else {
        ESP_LOGI(TAG, "MPU6050 detectado correctamente");
    }

    return ESP_OK;
}

void app_main(void)
{
    printf("=== FIRMWARE VIBRACIONES_EMT (MPU6050) INICIADO ===\n");
    fflush(stdout);

    esp_err_t ret;

    ESP_LOGI(TAG, "Inicializando I2C...");
    ret = i2c_master_init_debug();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo inicializando I2C, me quedo parado");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Inicializando MPU6050...");
    ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo inicializando MPU6050, me quedo parado");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Entrando en bucle de lectura de datos");

    while (1) {
        uint8_t data[14];
        ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 14);

        if (ret == ESP_OK) {
            int16_t ax = (int16_t)((data[0] << 8) | data[1]);
            int16_t ay = (int16_t)((data[2] << 8) | data[3]);
            int16_t az = (int16_t)((data[4] << 8) | data[5]);
            int16_t gx = (int16_t)((data[8] << 8) | data[9]);
            int16_t gy = (int16_t)((data[10] << 8) | data[11]);
            int16_t gz = (int16_t)((data[12] << 8) | data[13]);

            float ax_g = ax / ACCEL_SENS_2G;
            float ay_g = ay / ACCEL_SENS_2G;
            float az_g = az / ACCEL_SENS_2G;

            float gx_dps = gx / GYRO_SENS_250DPS;
            float gy_dps = gy / GYRO_SENS_250DPS;
            float gz_dps = gz / GYRO_SENS_250DPS;

            ESP_LOGI(TAG,
                     "Accel[g]: x=%.3f y=%.3f z=%.3f | Gyro[°/s]: x=%.3f y=%.3f z=%.3f",
                     ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        } else {
            ESP_LOGW(TAG, "Error leyendo MPU6050: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
    }
}
