/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include <stdlib.h>
#include <string.h>

#define APP_BUTTON (GPIO_NUM_0)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 38
#define I2C_MASTER_SCL_IO 39
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_ADDRESS 0x59

// I2C + joystick read logic from atoms3joy


#define RIGHTX 0
#define RIGHTY 1
#define LEFTX 2
#define LEFTY 3
#define RIGHT_STICK_BUTTON 0
#define LEFT_STICK_BUTTON 1
#define RIGHT_BUTTON 2
#define LEFT_BUTTON 3

#define LEFT_STICK_X_ADDRESS (0x00)
#define LEFT_STICK_Y_ADDRESS (0x02)
#define RIGHT_STICK_X_ADDRESS (0x20)
#define RIGHT_STICK_Y_ADDRESS (0x22)
#define LEFT_STICK_BUTTON_ADDRESS (0x70)
#define RIGHT_STICK_BUTTON_ADDRESS (0x71)
#define LEFT_BUTTON_ADDRESS (0x72)
#define RIGHT_BUTTON_ADDRESS (0x73)


uint16_t stick[4];
uint8_t button[4];
int16_t button_counter[4];
uint8_t button_state[4] = {0};
uint8_t button_old_state[4] = {0};
uint8_t buffer[4];

typedef struct __attribute__((packed, aligned(1))) {
  uint16_t buttons;
  int8_t leftXAxis;
  int8_t leftYAxis;
  int8_t rightXAxis;
  int8_t rightYAxis;
} HID_GamepadReport_Data_t;

HID_GamepadReport_Data_t _report;

static const char *TAG = "example";

#define TUSB_DESC_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(1))};

const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},    "Atom Joystick", "Atom Gamepad", "123456",
    "Gamepad HID Interface",
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16,
                       1)};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {}

static void i2c_master_init(void) {
  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = I2C_MASTER_SDA_IO,
                       .scl_io_num = I2C_MASTER_SCL_IO,
                       .sda_pullup_en = GPIO_PULLUP_ENABLE,
                       .scl_pullup_en = GPIO_PULLUP_ENABLE,
                       .master.clk_speed = I2C_MASTER_FREQ_HZ};
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

uint16_t read_2byte_data(uint8_t address) {
  uint8_t data[2] = {0};
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, address, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return data[1] << 8 | data[0];
}

uint8_t read_byte_data(uint8_t address) {
  uint8_t value = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, address, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &value, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return value;
}

void joy_update(void) {
  stick[RIGHTX] = read_2byte_data(RIGHT_STICK_X_ADDRESS);
  stick[RIGHTY] = read_2byte_data(RIGHT_STICK_Y_ADDRESS);
  stick[LEFTX] = read_2byte_data(LEFT_STICK_X_ADDRESS);
  stick[LEFTY] = read_2byte_data(LEFT_STICK_Y_ADDRESS);
  for (uint8_t i = 0; i < 4; i++) {
    button_state[i] = (~read_byte_data(LEFT_STICK_BUTTON_ADDRESS + i)) & 0x01;

    _report.buttons = (_report.buttons & ~((uint16_t)1 << i)) |
                      ((uint16_t)button_state[i] << i);
  }
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  if (x < in_min) {
    x = in_min;
  } else if (x > in_max) {
    x = in_max;
  }

  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
static void send_joystick_report(void) {
  joy_update();

  _report.leftXAxis =
      map(stick[LEFTX], 0, 4095, -127, 127); //(stick[LEFTX]  >> 4) & 0xFF;
  _report.leftYAxis =
      map(stick[LEFTY], 0, 4095, -127, 127); //(stick[LEFTY]  >> 4) & 0xFF;
  _report.rightXAxis =
      map(stick[RIGHTX], 0, 4095, -127, 127); //(stick[LEFTY] >> 4) & 0xFF;
  _report.rightYAxis =
      map(stick[RIGHTY], 0, 4095, -127, 127); //(stick[RIGHTY] >> 4) & 0xFF;

  tud_hid_report(1, &_report, sizeof(_report));
}

void app_main(void) {

  i2c_master_init();

  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t tusb_cfg = {
      .device_descriptor = NULL,
      .string_descriptor = hid_string_descriptor,
      .string_descriptor_count =
          sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
      .external_phy = false,
      .configuration_descriptor = hid_configuration_descriptor,
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "USB initialization DONE");

  while (1) {
    if (tud_mounted()) {
      send_joystick_report();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}