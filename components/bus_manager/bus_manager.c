#include "bus_manager.h"
#include "board_pins.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BUS_MGR";

static bool s_initialized = false;
static SemaphoreHandle_t s_spi_mutex = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;

// Park both chip-selects HIGH so a floating CS cannot grab the bus.
static esp_err_t park_chip_selects_high(void)
{
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << PIN_SG_CS) | (1ULL << PIN_LORA_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&cs_cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    gpio_set_level(PIN_SG_CS, 1);
    gpio_set_level(PIN_LORA_CS, 1);
    return ESP_OK;
}

static esp_err_t init_spi_bus(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };

    esp_err_t ret = spi_bus_initialize(BOARD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t init_i2c_bus(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(BOARD_I2C_PORT, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t bus_manager_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialising shared buses (SPI2, I2C0)...");

    s_spi_mutex = xSemaphoreCreateMutex();
    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_spi_mutex == NULL || s_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Could not create bus mutexes");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret;

    ret = park_chip_selects_high();
    if (ret != ESP_OK) return ret;

    ret = init_spi_bus();
    if (ret != ESP_OK) return ret;

    ret = init_i2c_bus();
    if (ret != ESP_OK) return ret;

    s_initialized = true;
    ESP_LOGI(TAG, "Buses ready.");
    return ESP_OK;
}

void bus_lock_spi(void)
{
    if (s_spi_mutex != NULL) {
        xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    }
}

void bus_unlock_spi(void)
{
    if (s_spi_mutex != NULL) {
        xSemaphoreGive(s_spi_mutex);
    }
}

void bus_lock_i2c(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
}

void bus_unlock_i2c(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}
