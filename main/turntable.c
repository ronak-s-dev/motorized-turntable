// ============================================================
// Includes
// ============================================================
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// ============================================================
// Function declarations
// ============================================================
void increase_or_decrease_rpm(int direction, int *rpm);
esp_err_t motor_set_step_frequency(uint32_t frequency_hz);
esp_err_t motor_start(uint32_t frequency_hz);
esp_err_t motor_stop(void);
uint32_t target_rpm_to_frequency(int target_rpm);

// ============================================================
// OLED Display Configuration
// ============================================================
// Globals
// ============================================================
// 5x7 Bitmap Font - Digits 0 to 9
// Each digit consists of 7 rows, with 5 bits representing
// the pixels in each row. A set bit represents an illuminated pixel.
// ============================================================

static const uint8_t digit_font[10][7] = {// 0
                                          {14, 17, 17, 17, 17, 17, 14},

                                          // 1
                                          {4, 12, 4, 4, 4, 4, 14},

                                          // 2
                                          {14, 17, 1, 2, 4, 8, 31},

                                          // 3
                                          {30, 1, 1, 14, 1, 1, 30},

                                          // 4
                                          {2, 6, 10, 18, 31, 2, 2},

                                          // 5
                                          {31, 16, 16, 30, 1, 1, 30},

                                          // 6
                                          {14, 16, 16, 30, 17, 17, 14},

                                          // 7
                                          {31, 1, 2, 4, 8, 8, 8},

                                          // 8
                                          {14, 17, 17, 14, 17, 17, 14},

                                          // 9
                                          {14, 17, 17, 15, 1, 1, 14}};

// SSD1306 display dimensions and framebuffer size.
// Each byte stores 8 vertically adjacent pixels.
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_BUFFER_SIZE (OLED_WIDTH * OLED_HEIGHT / 8)

// Software framebuffer representing the entire OLED display.
static uint8_t oled_buffer[OLED_BUFFER_SIZE];

// Clears the entire software framebuffer.
// Changes are sent to the physical display when oled_flush() is called.
void oled_clear(void) {
  for (int i = 0; i < OLED_BUFFER_SIZE; i++) {
    oled_buffer[i] = 0x00;
  }
}

// Updates an individual pixel in the framebuffer.
// Coordinates outside the display boundaries are ignored.
void oled_draw_pixel(int x, int y, bool on) {
  if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
    return;
  }

  int index = (y / 8) * OLED_WIDTH + x;
  uint8_t mask = 1 << (y % 8);

  if (on) {
    oled_buffer[index] |= mask;
  } else {
    oled_buffer[index] &= ~mask;
  }
}

// Draws a 5x7 digit at the specified framebuffer coordinates.
void oled_draw_digit(int x, int y, uint8_t digit) {
  if (digit > 9) {
    return;
  }

  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 5; col++) {
      if (digit_font[digit][row] & (1 << (4 - col))) {
        oled_draw_pixel(x + col, y + row, true);
      }
    }
  }
}

// Transfers the complete framebuffer to the SSD1306 display.
esp_err_t oled_flush(esp_lcd_panel_handle_t panel) {
  return esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT,
                                   oled_buffer);
}

// ============================================================
// Stepper Motor Configuration
// ============================================================

// STEP: GPIO32 - Receives pulse signals from MCPWM.
// DIR:  GPIO33 - Selects motor rotation direction.
// EN:   GPIO23 - Active LOW on the intended TMC2209 module.

#define MOTOR_STEP_GPIO GPIO_NUM_32
#define MOTOR_DIR_GPIO GPIO_NUM_33
#define MOTOR_EN_GPIO GPIO_NUM_23
#define MOTOR_MIN_STEP_FREQ_HZ 100
#define MOTOR_MAX_STEP_FREQ_HZ 600

// ============================================================
// Encoder Interrupt Handling
// ============================================================

// Counts encoder events that could not be placed in the queue.
// Useful for diagnosing queue congestion during development.

static volatile uint32_t encoder_queue_overflows = 0;

static void encoder_isr_handler(void *arg) {
  QueueHandle_t queue = (QueueHandle_t)arg;

  int clk_state = gpio_get_level(GPIO_NUM_25);
  int dt_state = gpio_get_level(GPIO_NUM_26);

  // Capture both quadrature signals and combine them into
  // a two-bit encoder state: CLK in bit 1, DT in bit 0.
  uint8_t encoder_state = (clk_state << 1) | dt_state;

  BaseType_t higher_priority_task_woken = pdFALSE;

  BaseType_t result =
      xQueueSendFromISR(queue, &encoder_state, &higher_priority_task_woken);

  if (result != pdPASS) {
    encoder_queue_overflows++;
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Timer Handle for Stepper Motor
static mcpwm_timer_handle_t motor_timer = NULL;
static mcpwm_oper_handle_t motor_operator = NULL;
static mcpwm_cmpr_handle_t motor_comparator = NULL;
static mcpwm_gen_handle_t motor_generator = NULL;

void app_main(void) {
  printf("Motorized Turntable Program Started....\n");

  esp_err_t err = ESP_OK;
  // ============================================================
  // Encoder queue
  // Stores captured quadrature states in FIFO order.
  // ============================================================
  QueueHandle_t encoder_queue = xQueueCreate(32, sizeof(uint8_t));
  if (encoder_queue == NULL) {
    printf("Failed to create encoder queue!\n");
    return;
  }

  // ============================================================
  // GPIO configuration
  // CLK: GPIO25
  // DT:  GPIO26
  // SW:  GPIO27
  // ============================================================
  gpio_reset_pin(GPIO_NUM_25);
  gpio_set_direction(GPIO_NUM_25, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_25, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(GPIO_NUM_25, GPIO_INTR_ANYEDGE);

  gpio_reset_pin(GPIO_NUM_26);
  gpio_set_direction(GPIO_NUM_26, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_26, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(GPIO_NUM_26, GPIO_INTR_ANYEDGE);

  gpio_reset_pin(GPIO_NUM_27);
  gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_27, GPIO_PULLUP_ONLY);

  // ============================================================
  // MCPWM Initialisation
  // ============================================================

  // Configure motor direction and enable signals as outputs.

  // Keep the driver disabled during initialisation.
  // TMC2209 EN is active LOW.

  gpio_set_direction(MOTOR_DIR_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_EN_GPIO, GPIO_MODE_OUTPUT);

  // Start with the motor driver disabled
  gpio_set_level(MOTOR_EN_GPIO, 1);

  // Set an initial direction
  gpio_set_level(MOTOR_DIR_GPIO, 0);

  // Configure a 1 MHz timer resolution.
  // Initial period: 10,000 ticks = 10 ms = 100 Hz.
  // The timer counts upward and updates its period at the
  // timer-empty event when a new period is requested.
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .period_ticks = 10000,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .flags.update_period_on_empty = true,
  };

  err = mcpwm_new_timer(&timer_config, &motor_timer);

  if (err != ESP_OK) {
    printf("Failed to create MCPWM timer: %s\n", esp_err_to_name(err));
    return;
  }

  // MCPWM operator configuration
  mcpwm_operator_config_t operator_config = {
      .group_id = 0,
  };

  err = mcpwm_new_operator(&operator_config, &motor_operator);

  if (err != ESP_OK) {
    printf("Failed to create MCPWM operator: %s\n", esp_err_to_name(err));
    return;
  }

  err = mcpwm_operator_connect_timer(motor_operator, motor_timer);

  if (err != ESP_OK) {
    printf("Failed to connect MCPWM operator to timer: %s\n",
           esp_err_to_name(err));
    return;
  }

  mcpwm_comparator_config_t comparator_config = {
      .flags.update_cmp_on_tez = true,
  };

  err = mcpwm_new_comparator(motor_operator, &comparator_config,
                             &motor_comparator);

  if (err != ESP_OK) {
    printf("Failed to create MCPWM comparator: %s\n", esp_err_to_name(err));
    return;
  }

  // MCPWM generator configuration
  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = MOTOR_STEP_GPIO,
  };

  err =
      mcpwm_new_generator(motor_operator, &generator_config, &motor_generator);

  if (err != ESP_OK) {
    printf("Failed to create MCPWM generator: %s\n", esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_timer_event(
      motor_generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    MCPWM_TIMER_EVENT_EMPTY,
                                                    MCPWM_GEN_ACTION_HIGH));

  if (err != ESP_OK) {
    printf("Failed to configure STEP rising edge: %s\n", esp_err_to_name(err));
    return;
  }

  err = mcpwm_generator_set_action_on_compare_event(
      motor_generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator,
                                     MCPWM_GEN_ACTION_LOW));

  if (err != ESP_OK) {
    printf("Failed to configure STEP falling edge: %s\n", esp_err_to_name(err));
    return;
  }

  err = mcpwm_comparator_set_compare_value(motor_comparator, 5000);

  if (err != ESP_OK) {
    printf("Failed to set comparator value: %s\n", esp_err_to_name(err));
    return;
  }

  err = mcpwm_timer_enable(motor_timer);

  if (err != ESP_OK) {
    printf("Failed to enable MCPWM timer: %s\n", esp_err_to_name(err));
    return;
  }

  // ============================================================
  // Initial encoder and button states
  // ============================================================
  // Locals
  int target_rpm = 0;
  int motor_running = 0;

  // Button debounce state
  int curr_btn_state = gpio_get_level(GPIO_NUM_27);
  int stable_btn_state = curr_btn_state;
  int candidate_btn_state = curr_btn_state;

  // Timestamp used to determine whether a button state is stable
  int64_t now_us = esp_timer_get_time();
  int64_t candidate_since_us = now_us;

  int prev_state =
      (gpio_get_level(GPIO_NUM_25) << 1) | gpio_get_level(GPIO_NUM_26);
  uint8_t encoder_info_rec_from_queue;

  // ============================================================
  // GPIO interrupt service and handler registration
  // ============================================================

  err = gpio_install_isr_service(0);

  if (err != ESP_OK) {
    printf("Failed to install GPIO ISR service: %s\n", esp_err_to_name(err));
    return;
  }

  err = gpio_isr_handler_add(GPIO_NUM_25, encoder_isr_handler,
                             (void *)encoder_queue);

  if (err != ESP_OK) {
    printf("Failed to register GPIO25 ISR: %s\n", esp_err_to_name(err));
    return;
  }

  err = gpio_isr_handler_add(GPIO_NUM_26, encoder_isr_handler,
                             (void *)encoder_queue);

  if (err != ESP_OK) {
    printf("Failed to register GPIO26 ISR: %s\n", esp_err_to_name(err));
    return;
  }

  // ============================================================
  // Quadrature decoder lookup table
  // Rows represent previous states.
  // Columns represent current states.
  // ============================================================
  int direction_table[4][4] = {
      {0, -1, +1, 0}, {+1, 0, 0, -1}, {-1, 0, 0, +1}, {0, +1, -1, 0}};

  // ============================================================
  // I2C Configurations
  // OLED SDA: GPIO21
  // OLED SCL: GPIO22
  // ============================================================
  // Bus Configuration
  i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = GPIO_NUM_21,
      .scl_io_num = GPIO_NUM_22,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t i2c_bus_handle;
  err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);

  if (err != ESP_OK) {
    printf("Failed to register bus handle: %s\n", esp_err_to_name(err));
    return;
  }

  printf("I2C bus initialised successfully!\n");

  // OLED Configuration
  // ============================================================
  // OLED: I2C Panel IO Configuration
  // ============================================================
  esp_lcd_panel_io_handle_t oled_io_handle = NULL;

  esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr = 0x3C,
      .scl_speed_hz = 100000,
      .control_phase_bytes = 1,
      .dc_bit_offset = 6,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .on_color_trans_done = NULL,
      .user_ctx = NULL,
      .flags =
          {
              .dc_low_on_data = false,
              .disable_control_phase = false,
          },
      .transaction_timeout_ms = 100,
  };

  err = esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_config, &oled_io_handle);

  if (err != ESP_OK) {
    printf("Failed to create OLED panel IO: %s\n", esp_err_to_name(err));
    return;
  }

  // ============================================================
  // OLED: SSD1306 Panel Configuration
  // ============================================================
  esp_lcd_panel_ssd1306_config_t ssd1306_config = {
      .height = OLED_HEIGHT,
      .contrast = 128,
  };

  esp_lcd_panel_dev_config_t panel_config = {
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
      .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
      .bits_per_pixel = 1,
      .reset_gpio_num = GPIO_NUM_NC,
      .vendor_config = &ssd1306_config,
  };

  esp_lcd_panel_handle_t oled_panel_handle = NULL;

  err = esp_lcd_new_panel_ssd1306(oled_io_handle, &panel_config,
                                  &oled_panel_handle);

  if (err != ESP_OK) {
    printf("Failed to create SSD1306 panel: %s\n", esp_err_to_name(err));
    return;
  }

  // ============================================================
  // OLED: Reset and Initialise
  // ============================================================

  oled_clear();
  err = oled_flush(oled_panel_handle);

  if (err != ESP_OK) {
    printf("OLED framebuffer transfer failed!\n");
    return;
  }

  printf("OLED panel initialised and switched on!\n");

  // ============================================================
  // Main application loop
  // Button debounce and encoder event processing.
  // ============================================================
  while (1) {
    curr_btn_state = gpio_get_level(GPIO_NUM_27);
    now_us = esp_timer_get_time();

    if (curr_btn_state != candidate_btn_state) {
      candidate_btn_state = curr_btn_state;
      candidate_since_us = now_us;
    } else if (candidate_btn_state != stable_btn_state &&
               (now_us - candidate_since_us) >=
                   100000) { // Accept a button state only after it remains
                             // unchanged for 100 ms
      stable_btn_state = candidate_btn_state;
      if (stable_btn_state == 0) {
        if (motor_running == 0) {
          if (target_rpm > 0) {
            uint32_t frequency_hz = target_rpm_to_frequency(target_rpm);
            esp_err_t err = motor_start(frequency_hz);

            if (err == ESP_OK) {
              motor_running = 1;
              printf("Motor started\n");
            } else {
              printf("Motor start failed: %s\n", esp_err_to_name(err));
            }

          } else {
            printf("Cannot start: target RPM is at zero\n");
          }
        } else {
          esp_err_t err = motor_stop();

          if (err == ESP_OK) {
            motor_running = 0;
            printf("Motor stopped\n");
          } else {
            printf("Motor stop failed: %s\n", esp_err_to_name(err));
          }
        }
      }
    }

    if (xQueueReceive(encoder_queue, &encoder_info_rec_from_queue,
                      pdMS_TO_TICKS(10)) == pdPASS) {
      // Process the received encoder state here
      if (prev_state != encoder_info_rec_from_queue) {
        int direction =
            direction_table[prev_state][encoder_info_rec_from_queue];
        // DEBUG LOG
        printf("Transition: %d -> %d | Direction: %+d\n", prev_state,
               encoder_info_rec_from_queue, direction);
        printf("State: %d | CLK: %d | DT: %d\n", encoder_info_rec_from_queue,
               (encoder_info_rec_from_queue >> 1) & 1,
               encoder_info_rec_from_queue & 1);
        switch (direction) {
        case +1:
        case -1:
          increase_or_decrease_rpm(direction, &target_rpm);
          break;
        default:
          if (prev_state != encoder_info_rec_from_queue) {
            printf("Invalid encoder transition: %d -> %d\n", prev_state,
                   encoder_info_rec_from_queue);
          }
          break;
        }
        if (direction == 1 || direction == -1) {
          if (motor_running && target_rpm > 0) {
            uint32_t frequency_hz = target_rpm_to_frequency(target_rpm);

            esp_err_t err = motor_set_step_frequency(frequency_hz);

            if (err != ESP_OK) {
              printf("Speed update failed: %s\n", esp_err_to_name(err));
            }
          } else if (motor_running && target_rpm == 0) {
            esp_err_t err = motor_stop();

            if (err == ESP_OK) {
              motor_running = 0;
              printf("Target RPM reached zero. Motor stopped.\n");
            } else {
              printf("Motor stop failed: %s\n", esp_err_to_name(err));
            }
          }
        }

        prev_state = encoder_info_rec_from_queue;
      }
    }
  }
}

// Accumulates four valid quadrature transitions
// before changing the target RPM by one.
void increase_or_decrease_rpm(int direction, int *rpm) {
  // Retains the transition count between function calls
  static int transition_count = 0;

  // Accumulate the encoder's direction
  if (direction == 1) {
    transition_count++;
  } else if (direction == -1) {
    transition_count--;
  }

  printf("Transition accumulator: %d\n", transition_count);

  if (transition_count >= 4) {
    if (*rpm < 100) {
      *rpm += 1;
    }

    transition_count = 0;
  } else if (transition_count <= -4) {
    if (*rpm > 0) {
      *rpm -= 1;
    }
    transition_count = 0;
  }
  printf("Target RPM: %d\n", *rpm);
}

esp_err_t motor_set_step_frequency(uint32_t frequency_hz) {
  if (frequency_hz < MOTOR_MIN_STEP_FREQ_HZ ||
      frequency_hz > MOTOR_MAX_STEP_FREQ_HZ) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t period_ticks = 1000000 / frequency_hz;
  uint32_t compare_ticks = period_ticks / 2;

  esp_err_t err = mcpwm_timer_set_period(motor_timer, period_ticks);

  if (err != ESP_OK) {
    return err;
  }

  err = mcpwm_comparator_set_compare_value(motor_comparator, compare_ticks);

  return err;
}

// Helper Functions for Motor Start / Stop Functionality
esp_err_t motor_start(uint32_t frequency_hz) {
  esp_err_t err = motor_set_step_frequency(frequency_hz);

  if (err != ESP_OK) {
    return err;
  }

  err = gpio_set_level(MOTOR_EN_GPIO, 0);

  if (err != ESP_OK) {
    return err;
  }

  err = mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP);

  if (err != ESP_OK) {
    gpio_set_level(MOTOR_EN_GPIO, 1);
    return err;
  }

  return ESP_OK;
}

esp_err_t motor_stop(void) {
  esp_err_t err = mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_STOP_EMPTY);

  if (err != ESP_OK) {
    return err;
  }

  err = gpio_set_level(MOTOR_EN_GPIO, 1);

  return err;
}

uint32_t target_rpm_to_frequency(int target_rpm) {
  return 100 + (target_rpm * 5);
}
