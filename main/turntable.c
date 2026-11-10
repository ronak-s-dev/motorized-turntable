// ============================================================
// Includes
// ============================================================
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>

// ============================================================
// Function declarations
// ============================================================
void increase_or_decrease_rpm(int direction, int *rpm);

static volatile uint32_t encoder_queue_overflows = 0;

static void encoder_isr_handler(void *arg) {
  QueueHandle_t queue = (QueueHandle_t)arg;

  int clk_state = gpio_get_level(GPIO_NUM_25);
  int dt_state = gpio_get_level(GPIO_NUM_26);

  uint8_t encoder_state = (clk_state << 1) | dt_state;

  BaseType_t higher_priority_task_woken = pdFALSE;

  BaseType_t result =
      xQueueSendFromISR(queue, &encoder_state, &higher_priority_task_woken);

  if (result != pdPASS) {
    encoder_queue_overflows++;
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void app_main(void) {
  printf("Motorized Turntable Program Started....\n");

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
  // Initial encoder and button states
  // ============================================================
  // // Locals
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
  esp_err_t err = gpio_install_isr_service(0);

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
        motor_running = !motor_running;
        printf("Motor state: %s\n", motor_running ? "RUNNING" : "STOPPED");
      }
    }

    if (xQueueReceive(encoder_queue, &encoder_info_rec_from_queue,
                      pdMS_TO_TICKS(10)) == pdPASS) {
      // Process the received encoder state here
      if (prev_state != encoder_info_rec_from_queue) {
        int direction =
            direction_table[prev_state][encoder_info_rec_from_queue];
        switch (direction) {
        case +1:
          increase_or_decrease_rpm(direction, &target_rpm);
          break;
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
