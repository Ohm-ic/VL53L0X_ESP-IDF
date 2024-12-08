#ifndef VL53L0X_H
#define VL53L0X_H

#include "driver/i2c.h"
#include "esp_err.h"

#define VL53L0X_I2C_ADDR           (0x29)
#define VL53L0X_REG_IDENTIFICATION  (0xC0)
#define VL53L0X_REG_SYSTEM_RANGE    (0x00)

// Possible ranging modes
typedef enum {
    VL53L0X_MODE_SINGLE_RANGING,
    VL53L0X_MODE_CONTINUOUS_RANGING,
    VL53L0X_MODE_AUTONOMOUS
} vl53l0x_mode_t;

// Configuration structure
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    vl53l0x_mode_t mode;
    uint16_t range_mm;
} vl53l0x_config_t;

// Function prototypes
esp_err_t vl53l0x_init(vl53l0x_config_t *config);
esp_err_t vl53l0x_start_measurement(vl53l0x_config_t *config);
esp_err_t vl53l0x_read_range(vl53l0x_config_t *config, uint16_t *range_mm);
esp_err_t vl53l0x_stop_measurement(vl53l0x_config_t *config);

#endif // VL53L0X_H