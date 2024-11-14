/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_io_expander_tca9554.h"

#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#include "esp_lcd_touch_gt911.h"

/* LCD size */
#define BOARD_LCD_H_RES (320)
#define BOARD_LCD_V_RES (240)

/* LCD settings */
#define BOARD_LCD_SPI_NUM          (SPI3_HOST)
#define BOARD_LCD_PIXEL_CLK_HZ     (40 * 1000 * 1000)
#define BOARD_LCD_CMD_BITS         (8)
#define BOARD_LCD_PARAM_BITS       (8)
#define BOARD_LCD_COLOR_SPACE      (ESP_LCD_COLOR_SPACE_BGR)
#define BOARD_LCD_BITS_PER_PIXEL   (16)
#define BOARD_LCD_DRAW_BUFF_DOUBLE (1)
#define BOARD_LCD_DRAW_BUFF_HEIGHT (50)
#define BOARD_LCD_BL_ON_LEVEL      (1)

/* LCD pins */
#define BOARD_LCD_GPIO_SCLK (GPIO_NUM_1)
#define BOARD_LCD_GPIO_MOSI (GPIO_NUM_0)
#define BOARD_LCD_GPIO_RST  (GPIO_NUM_NC) /* TCA9554 P2 */
#define BOARD_LCD_GPIO_DC   (GPIO_NUM_2)
#define BOARD_LCD_GPIO_CS   (GPIO_NUM_46)
#define BOARD_LCD_GPIO_BL   (GPIO_NUM_NC) /* TCA9554 P0 */

/* Touch settings */
#define BOARD_TOUCH_I2C_NUM    (0)
#define BOARD_TOUCH_I2C_CLK_HZ (400000)

/* LCD touch pins */
#define BOARD_TOUCH_I2C_SCL  (GPIO_NUM_18)
#define BOARD_TOUCH_I2C_SDA  (GPIO_NUM_17)
#define BOARD_TOUCH_GPIO_INT (GPIO_NUM_NC) /* TCA9554 P4 */

/* TCA9554 ctrl pins */
#define BOARD_TOUCH_TCA9554_GPIO_INT BIT(4)
#define BOARD_LCD_TCA9554_GPIO_RST   BIT(2)
#define BOARD_LCD_TCA9554_GPIO_BL    BIT(0)
#define BOARD_TCA9554_CTRL_PINS      (BOARD_LCD_TCA9554_GPIO_BL | BOARD_LCD_TCA9554_GPIO_RST | BOARD_TOUCH_TCA9554_GPIO_INT)

static const char *TAG = "EXAMPLE";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/* i2c handle */
i2c_master_bus_handle_t i2c0_handle = NULL;

/* IO expander */
static esp_io_expander_handle_t io_expander = NULL;

static esp_err_t app_iic_init(void)
{
    i2c_master_bus_config_t i2c1_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = BOARD_TOUCH_I2C_SCL,
        .sda_io_num = BOARD_TOUCH_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c1_mst_config, &i2c0_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

    return ret;
}

static esp_err_t app_tca9554_init(void)
{
    esp_err_t ret;

    ret = esp_io_expander_new_i2c_tca9554(i2c0_handle, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "TCA9554 create returned error");

    if (ESP_OK != ret) {
        return ret;
    }

    ret = esp_io_expander_set_dir(io_expander, BOARD_TCA9554_CTRL_PINS, IO_EXPANDER_OUTPUT);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "TCA9554 init pins error");

    if (ESP_OK != ret) {
        return ret;
    }

    // Print state
    ret = esp_io_expander_print_state(io_expander);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    esp_io_expander_set_level(io_expander, BOARD_LCD_TCA9554_GPIO_BL, 0);
    esp_io_expander_set_level(io_expander, BOARD_LCD_TCA9554_GPIO_RST, 0);
    esp_io_expander_set_level(io_expander, BOARD_TOUCH_TCA9554_GPIO_INT, 0);

    return ESP_OK;
}

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    // gpio_config_t bk_gpio_config = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << BOARD_LCD_GPIO_BL };
    // ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BOARD_LCD_GPIO_SCLK,
        .mosi_io_num = BOARD_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BOARD_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_LCD_GPIO_DC,
        .cs_gpio_num = BOARD_LCD_GPIO_CS,
        .pclk_hz = BOARD_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
        .lcd_param_bits = BOARD_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BOARD_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_LCD_GPIO_RST,
        .color_space = BOARD_LCD_COLOR_SPACE,
        .bits_per_pixel = BOARD_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);

    esp_io_expander_set_level(io_expander, BOARD_LCD_TCA9554_GPIO_RST, 0);
    esp_io_expander_set_level(io_expander, BOARD_TOUCH_TCA9554_GPIO_INT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, BOARD_TOUCH_TCA9554_GPIO_INT, 1);
    vTaskDelay(pdMS_TO_TICKS(6));
    ret = esp_io_expander_set_dir(io_expander, BOARD_TOUCH_TCA9554_GPIO_INT, IO_EXPANDER_INPUT);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "TCA9554 set touch pin to input is error");

    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, BOARD_LCD_TCA9554_GPIO_BL, BOARD_LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(BOARD_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_touch_init(void)
{
    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BOARD_LCD_H_RES,
        .y_max = BOARD_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
        .int_gpio_num = BOARD_TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c0_handle, &tp_io_config, &tp_io_handle), TAG, "");

    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &touch_handle);
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 4096,       /* LVGL task stack size */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = { .io_handle = lcd_io,
                                               .panel_handle = lcd_panel,
                                               .buffer_size = BOARD_LCD_H_RES * BOARD_LCD_DRAW_BUFF_HEIGHT,
                                               .double_buffer = BOARD_LCD_DRAW_BUFF_DOUBLE,
                                               .hres = BOARD_LCD_H_RES,
                                               .vres = BOARD_LCD_V_RES,
                                               .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
                                               .color_format = LV_COLOR_FORMAT_RGB565,
#endif
                                               .rotation = {
                                                   .swap_xy = false,
                                                   .mirror_x = true,
                                                   .mirror_y = true,
                                               },
                                               .flags = {
                                                   .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
                                                   .swap_bytes = true,
#endif
                                               } };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

static void _app_button_cb(lv_event_t *e)
{
    lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
    rotation++;
    if (rotation > LV_DISPLAY_ROTATION_270) {
        rotation = LV_DISPLAY_ROTATION_0;
    }

    /* LCD HW rotation */
    lv_disp_set_rotation(lvgl_disp, rotation);
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* Your LVGL objects code here .... */

    /* Create image */
    lv_obj_t *img_logo = lv_img_create(scr);
    lv_img_set_src(img_logo, &esp_logo);
    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, BOARD_LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#if LVGL_VERSION_MAJOR == 8
    lv_label_set_recolor(label, true);
    lv_label_set_text(
        label, "#FF0000 " LV_SYMBOL_BELL " Hello world Espressif and LVGL " LV_SYMBOL_BELL "#\n#FF9400 " LV_SYMBOL_WARNING
               " For simplier initialization, use BSP " LV_SYMBOL_WARNING " #");
#else
    lv_label_set_text(
        label, LV_SYMBOL_BELL " Hello world Espressif and LVGL " LV_SYMBOL_BELL "\n " LV_SYMBOL_WARNING " For simplier initialization, use BSP " LV_SYMBOL_WARNING);
#endif
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    /* Button */
    lv_obj_t *btn = lv_btn_create(scr);
    label = lv_label_create(btn);
    lv_label_set_text_static(label, "Rotate screen");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    ESP_ERROR_CHECK(app_iic_init());

    ESP_ERROR_CHECK(app_tca9554_init());

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* Touch initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}
