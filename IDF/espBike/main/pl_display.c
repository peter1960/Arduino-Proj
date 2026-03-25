#include "pins.h"
#include "pl_display.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"

#define EPD_WIDTH 250
#define EPD_HEIGHT 122
#define EPD_WIDTH_BYTES ((EPD_WIDTH + 7) / 8) // = 32

static spi_device_handle_t spi;
void epd_spi_init();
void epd_send_command(uint8_t cmd);
void epd_send_data(uint8_t data);
void epd_wait_busy();
void epd_reset();
void epd_init();
void epd_clear();
void epd_display(const uint8_t *buffer);
void fill_test_pattern();
// --- LOW LEVEL ---
void test_epd()
{
    static uint8_t buffer[EPD_WIDTH_BYTES * EPD_HEIGHT];

    epd_spi_init();
    epd_init();

    // Clear twice (important for clean panel)
    epd_clear();
    epd_clear();

    // Fill test pattern
    for (int y = 0; y < EPD_HEIGHT; y++)
    {
        for (int x = 0; x < EPD_WIDTH_BYTES; x++)
        {
            if (y % 2 == 0)
                buffer[y * EPD_WIDTH_BYTES + x] = 0xAA;
            else
                buffer[y * EPD_WIDTH_BYTES + x] = 0x55;
        }
    }

    epd_display(buffer);
}
// ---------------- DISPLAY ----------------
#define EPD_WIDTH 250
#define EPD_HEIGHT 122
#define EPD_WIDTH_BYTES ((EPD_WIDTH + 7) / 8) // = 32

static spi_device_handle_t spi;

// ---------------- LOW LEVEL ----------------

void epd_send_command(uint8_t cmd)
{
    gpio_set_level(PIN_EPD_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(spi, &t);
}

void epd_send_data(uint8_t data)
{
    gpio_set_level(PIN_EPD_DC, 1);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    spi_device_transmit(spi, &t);
}

void epd_wait_busy()
{
    // Try 1 first (most common)
    while (gpio_get_level(PIN_EPD_BUSY) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void epd_reset()
{
    gpio_set_level(PIN_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// ---------------- INIT ----------------

void epd_set_window()
{
    // X range (bytes!)
    epd_send_command(0x44);
    epd_send_data(0x00);
    epd_send_data(EPD_WIDTH_BYTES - 1);

    // Y range
    epd_send_command(0x45);
    epd_send_data(0x00);
    epd_send_data(0x00);
    epd_send_data((EPD_HEIGHT - 1) & 0xFF);
    epd_send_data((EPD_HEIGHT - 1) >> 8);
}

void epd_set_cursor()
{
    // X pointer
    epd_send_command(0x4E);
    epd_send_data(0x00);

    // Y pointer
    epd_send_command(0x4F);
    epd_send_data(0x00);
    epd_send_data(0x00);
}

void epd_init()
{
    epd_reset();

    epd_send_command(0x01); // DRIVER_OUTPUT_CONTROL
    epd_send_data((EPD_HEIGHT - 1) & 0xFF);
    epd_send_data((EPD_HEIGHT - 1) >> 8);
    epd_send_data(0x00);

    epd_send_command(0x11); // DATA_ENTRY_MODE
    epd_send_data(0x03);    // X+, Y+

    epd_set_window();
    epd_set_cursor();

    epd_wait_busy();
}

// ---------------- DISPLAY ----------------

void epd_update()
{
    epd_send_command(0x22);
    epd_send_data(0xF7);
    epd_send_command(0x20);
    epd_wait_busy();
}

void epd_clear()
{
    epd_set_window();
    epd_set_cursor();

    epd_send_command(0x24);

    for (int i = 0; i < (EPD_WIDTH_BYTES * EPD_HEIGHT); i++)
    {
        epd_send_data(0xFF); // white
    }

    epd_update();
}

void epd_display(const uint8_t *buffer)
{
    epd_set_window();
    epd_set_cursor();

    epd_send_command(0x24);

    for (int i = 0; i < (EPD_WIDTH_BYTES * EPD_HEIGHT); i++)
    {
        epd_send_data(buffer[i]);
    }

    epd_update();
}

// ---------------- SPI INIT ----------------

void epd_spi_init()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_EPD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_EPD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2MHz safe
        .mode = 0,
        .spics_io_num = PIN_EPD_CS,
        .queue_size = 1,
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    gpio_set_direction(PIN_EPD_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_EPD_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_EPD_BUSY, GPIO_MODE_INPUT);
}

// ---------------- TEST PATTERN ----------------

static uint8_t buffer[EPD_WIDTH_BYTES * EPD_HEIGHT];

void fill_test_pattern()
{
    for (int y = 0; y < EPD_HEIGHT; y++)
    {
        for (int x = 0; x < EPD_WIDTH_BYTES; x++)
        {
            if (y % 2 == 0)
                buffer[y * EPD_WIDTH_BYTES + x] = 0xAA; // 10101010
            else
                buffer[y * EPD_WIDTH_BYTES + x] = 0x55; // 01010101
        }
    }
}
