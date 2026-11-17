#include "bus_manager.h"
#include "board_pins.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "BUS_MGR";
static bool s_initialized = false;

static esp_err_t board_init_safe_pins(void) {
    const uint64_t pyro_out_mask = (1ULL << PIN_IGNITE1) | (1ULL << PIN_IGNITE2);
    gpio_config_t pyro_cfg = {
        .pin_bit_mask = pyro_out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pyro_cfg), TAG, "Pyro config err");
    gpio_set_level(PIN_IGNITE1, 0);
    gpio_set_level(PIN_IGNITE2, 0);

    const uint64_t cs_out_mask = (1ULL << PIN_SG_CS) | (1ULL << PIN_LORA_CS);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = cs_out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cs_cfg), TAG, "CS config err");
    gpio_set_level(PIN_SG_CS, 1);
    gpio_set_level(PIN_LORA_CS, 1);

    gpio_config_t drdy_cfg = {
        .pin_bit_mask = (1ULL << PIN_SG_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&drdy_cfg), TAG, "SG_DRDY config err");

    return ESP_OK;
}

static esp_err_t init_spi_bus(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(BOARD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
        TAG,
        "spi_bus_initialize (SPI2) err"
    );

    return ESP_OK;
}

static esp_err_t init_i2c_bus(void) {
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(
        i2c_param_config(BOARD_I2C_PORT, &i2c_cfg), 
        TAG, 
        "i2c_param_config (I2C) err"
    );

    ESP_RETURN_ON_ERROR(
        i2c_driver_install(BOARD_I2C_PORT, i2c_cfg.mode, 0, 0, 0),
        TAG,
        "i2c_driver_install (I2C) err"
    );

    return ESP_OK;
}

static esp_err_t init_gnss_uart(void) {
    const uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(BOARD_GNSS_UART_PORT, 1024, 0, 0, NULL, 0),
        TAG,
        "uart_driver_install (GNSS) err"
    );

    ESP_RETURN_ON_ERROR(
        uart_param_config(BOARD_GNSS_UART_PORT, &uart_cfg),
        TAG,
        "uart_param_config (GNSS) err"
    );

    ESP_RETURN_ON_ERROR(
        uart_set_pin(BOARD_GNSS_UART_PORT, PIN_GNSS_UART_TX, PIN_GNSS_UART_RX, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        TAG,
        "uart_set_pin (GNSS) err"
    );

    return ESP_OK;
}

esp_err_t bus_manager_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Safe state pins and buses init...");

    ESP_RETURN_ON_ERROR(
        board_init_safe_pins(),
        TAG,
        "safe_pins err"
    );

    ESP_RETURN_ON_ERROR(
        init_spi_bus(),
        TAG,
        "SPI2 err"
    );

    ESP_RETURN_ON_ERROR(
        init_i2c_bus(),
        TAG,
        "I2C0 err"
    );

    ESP_RETURN_ON_ERROR(
        init_gnss_uart(),
        TAG,
        "GNSS UART1 err"
    );

    s_initialized = true;
    ESP_LOGI(TAG, "Buses init succesfully (SPI2, I2C0, UART1)");
    return ESP_OK;
}

esp_err_t bus_manager_spi_acquire(spi_device_handle_t dev_handle, TickType_t wait_ticks) {
    return spi_device_acquire_bus(dev_handle, wait_ticks);
}

void bus_manager_spi_release(spi_device_handle_t dev_handle) {
    spi_device_release_bus(dev_handle);
}