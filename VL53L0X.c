#include "vl53l0x.h"
#include "esp_log.h"

static const char *TAG = "VL53L0X";

// Helper function to write to a register
static esp_err_t vl53l0x_write_reg(vl53l0x_config_t *config, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (config->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg, true);
    ret |= i2c_master_write(cmd, data, len, true);
    ret |= i2c_master_stop(cmd);
    
    ret |= i2c_master_cmd_begin(config->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Helper function to read from a register
static esp_err_t vl53l0x_read_reg(vl53l0x_config_t *config, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (config->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg, true);
    
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (config->i2c_addr << 1) | I2C_MASTER_READ, true);
    ret |= i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    ret |= i2c_master_stop(cmd);
    
    ret |= i2c_master_cmd_begin(config->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Initialize the VL53L0X sensor
esp_err_t vl53l0x_init(vl53l0x_config_t *config) {
    ESP_LOGI(TAG, "Initializing VL53L0X sensor");
    
    // Verify device ID
    uint8_t device_id;
    esp_err_t ret = vl53l0x_read_reg(config, VL53L0X_REG_IDENTIFICATION, &device_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }
    
    // Configure sensor based on selected mode
    switch (config->mode) {
        case VL53L0X_MODE_SINGLE_RANGING:
            // Configure for single shot mode
            break;
        case VL53L0X_MODE_CONTINUOUS_RANGING:
            // Configure for continuous ranging
            break;
        case VL53L0X_MODE_AUTONOMOUS:
            // Configure for autonomous mode
            break;
        default:
            ESP_LOGE(TAG, "Invalid ranging mode");
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

// Start a measurement
esp_err_t vl53l0x_start_measurement(vl53l0x_config_t *config) {
    ESP_LOGI(TAG, "Starting VL53L0X measurement");
    
    // Trigger measurement based on mode
    uint8_t cmd = 0x01; // Example start command
    return vl53l0x_write_reg(config, VL53L0X_REG_SYSTEM_RANGE, &cmd, 1);
}

// Read range measurement
esp_err_t vl53l0x_read_range(vl53l0x_config_t *config, uint16_t *range_mm) {
    uint8_t range_data[2];
    esp_err_t ret = vl53l0x_read_reg(config, VL53L0X_REG_SYSTEM_RANGE, range_data, 2);
    
    if (ret == ESP_OK) {
        *range_mm = (range_data[0] << 8) | range_data[1];
        ESP_LOGI(TAG, "Range measurement: %d mm", *range_mm);
    }
    
    return ret;
}

// Stop measurement
esp_err_t vl53l0x_stop_measurement(vl53l0x_config_t *config) {
    ESP_LOGI(TAG, "Stopping VL53L0X measurement");
    
    // Stop measurement command
    uint8_t cmd = 0x00; // Example stop command
    return vl53l0x_write_reg(config, VL53L0X_REG_SYSTEM_RANGE, &cmd, 1);
}