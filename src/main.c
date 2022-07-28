#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// LED BUILT-IN
#define BUILT_IN_LED 33

// UART PIN
#define UART_PIN_TXD GPIO_NUM_1
#define UART_PIN_RXD GPIO_NUM_3
#define UART_PIN_RTS UART_PIN_NO_CHANGE
#define UART_PIN_CTS UART_PIN_NO_CHANGE
#define UART_BUF_SIZE 1024

// CAMERA PIN
#define CAM_PIN_PWDN GPIO_NUM_32  // power down is not used
#define CAM_PIN_RESET -1          // software reset will be performed
#define CAM_PIN_XCLK GPIO_NUM_0
#define CAM_PIN_SIOD GPIO_NUM_26
#define CAM_PIN_SIOC GPIO_NUM_27
#define CAM_PIN_D7 GPIO_NUM_35
#define CAM_PIN_D6 GPIO_NUM_34
#define CAM_PIN_D5 GPIO_NUM_39
#define CAM_PIN_D4 GPIO_NUM_36
#define CAM_PIN_D3 GPIO_NUM_21
#define CAM_PIN_D2 GPIO_NUM_19
#define CAM_PIN_D1 GPIO_NUM_18
#define CAM_PIN_D0 GPIO_NUM_5
#define CAM_PIN_VSYNC GPIO_NUM_25
#define CAM_PIN_HREF GPIO_NUM_23
#define CAM_PIN_PCLK GPIO_NUM_22

static uart_config_t konfig_uart = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static camera_config_t konfig_kamera = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 16000000,  // EXPERIMENTAL: Set to 16MHz on ESP32-S2 or
                               // ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,  // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size =
        FRAMESIZE_UXGA,  // QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12,  // 0-63 lower number means higher quality
    .fb_count =
        1,  // if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY  // CAMERA_GRAB_LATEST. Sets when
                                         // buffers should be filled
};

esp_err_t mulai_uart() {
  esp_err_t err =
      uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
  if (err == ESP_OK) {
    uart_param_config(UART_NUM_0, &konfig_uart);
    uart_set_pin(UART_NUM_0, UART_PIN_TXD, UART_PIN_RXD, UART_PIN_RTS,
                 UART_PIN_CTS);
  }
  return err;
}

esp_err_t mulai_kamera() {
  // initialize the camera
  esp_err_t err = esp_camera_init(&konfig_kamera);
  return err;
}

esp_err_t mulai_led() {
  gpio_pad_select_gpio(BUILT_IN_LED);
  return gpio_set_direction(BUILT_IN_LED, GPIO_MODE_OUTPUT);
}

esp_err_t proses_gambar(size_t lebar, size_t tinggi, pixformat_t format,
                        uint8_t *fb, size_t panjang) {
  return ESP_OK;
}

void app_main() {
  mulai_led();
  mulai_uart();
  camera_fb_t *pic;
  esp_err_t errInitKamera = mulai_kamera();
  if (errInitKamera == ESP_OK) {
    uart_write_bytes(UART_NUM_0, "\nKAMERA BERHASIL DIMULAI!\n", 26);
  } else {
    uart_write_bytes(UART_NUM_0, "\nKAMERA GAGAL DIMULAI!\n", 23);
  }
  uint8_t i = 0;
  TickType_t t0 = 0;
  TickType_t tNow = 0;
  char picBufSize[8];
  uint8_t *buf = (uint8_t[UART_BUF_SIZE]){'<', ' '};
  uint8_t *rawInp = buf + 2;
  uart_write_bytes(UART_NUM_0, "\nMULAI\n", 7);

  while (1) {
    if (tNow - t0 > 500) {
      uint8_t len = uart_read_bytes(UART_NUM_0, (void *)rawInp, UART_BUF_SIZE,
                                    20 / portTICK_RATE_MS);
      buf[len + 2] = '\0';  // null-terminated string
      gpio_set_level(BUILT_IN_LED, i % 2);
      if (len > 0) {
        uart_write_bytes(UART_NUM_0, (const void *)buf,
                         strlen((const char *)rawInp) + 2);
        if (strcmp((const char *)rawInp, "tangkap\n") == 0) {
          if (errInitKamera == ESP_OK) {
            pic = esp_camera_fb_get();
            if (!pic) {
              uart_write_bytes(UART_NUM_0, "GAG\n", 4);
            } else {
              uart_write_bytes(UART_NUM_0, "SUK\n", 4);
              memset(&picBufSize[0], 0, sizeof(picBufSize));
              sprintf(picBufSize, "%d", pic->len);
              uart_write_bytes(UART_NUM_0, picBufSize, sizeof(picBufSize));
              uart_write_bytes(UART_NUM_0, pic->buf, pic->len);
              esp_camera_fb_return(pic);
            }
          } else {
            uart_write_bytes(UART_NUM_0, "KAMERA GAGAL MULAI!\n", 19);
          }
        }
      }
      i++;
      t0 = tNow;
    }
    tNow = xTaskGetTickCount();
    vTaskDelay(1 / portTICK_RATE_MS);
  }
}