/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 * Modifications Copyright 2025 nuovotaka
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zmk/keymap.h>
#include <zephyr/init.h>

// Utility macros
#ifndef CLAMP
#define CLAMP(val, low, high)                                                  \
  (((val) < (low)) ? (low) : (((val) > (high)) ? (high) : (val)))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#include "paw3222.h"
#include "paw3222_input.h"
#include "paw3222_power.h"
#include "paw3222_regs.h"
#include "paw3222_spi.h"

LOG_MODULE_DECLARE(paw32xx);

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

/* Idle (no-motion) handling
 * - If no motion activity for PAW32XX_IDLE_TIMEOUT_SECONDS, enter idle:
 *   disable IRQ, cancel motion work/timer.  Lightweight idle (sensor not fully powered down).
 * - Wake on motion IRQ or motion activity: re-enable IRQ, restart motion work/timer.
 */
#define PAW32XX_IDLE_TIMEOUT_SECONDS 300 /* 5 minutes */

static struct k_timer paw32xx_idle_timer;
static const struct device *paw32xx_idle_dev;
static volatile bool paw32xx_idle = false;
static volatile bool paw32xx_idle_timer_inited = false;

void paw32xx_idle_timeout_handler(struct k_timer *timer);
void paw32xx_idle_enter(const struct device *dev);
void paw32xx_idle_exit(const struct device *dev);

extern struct k_timer bothscroll_key_timer;
  /* XYSCROLL_DEBUG_LOG */
//  LOG_INF("input_mode: %d", input_mode); // XYSCROLL_DEBUG_LOG
  /* XYSCROLL_DEBUG_LOG */
//  LOG_INF("delta_x: %d, delta_y: %d", delta_x, delta_y); // XYSCROLL_DEBUG_LOG

/**
 * @brief Calculate absolute value of int16_t (memory optimized)
 *
 * Computes the absolute value of a 16-bit signed integer without using
 * the standard library abs() function. This inline function is optimized
 * for memory usage and performance in the motion processing path.
 *
 * @param value Input signed 16-bit integer
 * 
 * @return Absolute value of the input
 * 
 * @note This function handles the INT16_MIN case correctly by returning
 *       the positive equivalent without overflow issues.
 */
static inline int16_t abs_int16(int16_t value) {
  return (value < 0) ? -value : value;
}

/**
 * @brief Safely add to scroll accumulator with overflow protection
 *
 * Adds a delta value to the scroll accumulator while preventing overflow.
 * Clamps the result to INT16_MIN/INT16_MAX range and logs warnings on overflow.
 *
 * @param accumulator Pointer to the current accumulator value
 * @param delta Delta value to add
 * 
 * @note Modifies the accumulator value in place
 */
static inline void add_to_scroll_accumulator(int16_t *accumulator, int16_t delta) {
  int32_t temp = (int32_t)*accumulator + delta;
  if (temp > INT16_MAX) {
    LOG_WRN("Scroll accumulator overflow: %d, clamped to %d", temp, INT16_MAX);
    *accumulator = INT16_MAX;
  } else if (temp < INT16_MIN) {
    LOG_WRN("Scroll accumulator underflow: %d, clamped to %d", temp, INT16_MIN);
    *accumulator = INT16_MIN;
  } else {
    *accumulator = (int16_t)temp;
  }
}

/**
 * @brief Process scroll input and generate scroll events
 *
 * Accumulates scroll movement and generates scroll events when threshold is reached.
 * Handles both vertical and horizontal scrolling based on the input type.
 *
 * @param dev Device pointer for input reporting
 * @param accumulator Pointer to scroll accumulator
 * @param scroll_delta Scroll movement delta
 * @param threshold Threshold for triggering scroll events
 * @param is_horizontal true for horizontal scroll, false for vertical
 */
static void process_scroll_input(const struct device *dev, int16_t *accumulator, 
                                int16_t scroll_delta, uint8_t threshold, bool is_horizontal) {
  add_to_scroll_accumulator(accumulator, scroll_delta);
  
  if (abs_int16(*accumulator) >= threshold) {
    int16_t scroll_direction = (*accumulator > 0) ? 1 : -1;
    uint16_t input_code = is_horizontal ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL;
    
    input_report_rel(dev, input_code, scroll_direction, true, K_FOREVER);
    *accumulator -= scroll_direction * threshold;
  }
}

enum paw32xx_input_mode
get_input_mode_for_current_layer(const struct device *dev) {
  const struct paw32xx_config *cfg = dev->config;
  struct paw32xx_data *data = dev->data;

// LOG_INF("cfg->switch_method=%d, data->current_mode=%d", cfg->switch_method, data->current_mode); // XYSCROLL_DEBUG_LOG

// enum paw32xx_mode_switch_method => cfg->switch_method
//  0: PAW32XX_SWITCH_LAYER,  /**< Original layer-based switching using ZMK layers */
//  1: PAW32XX_SWITCH_TOGGLE, /**< Toggle key based switching using behavior API *

// enum paw32xx_current_mode => data->current_mode
//  0: PAW32XX_MODE_MOVE,                    /**< Standard cursor movement mode */
//  1: PAW32XX_MODE_SCROLL,                  /**< Vertical scrolling mode */
//  2: PAW32XX_MODE_SCROLL_HORIZONTAL,       /**< Horizontal scrolling mode */
//  3: PAW32XX_MODE_SNIPE,                   /**< High-precision cursor movement mode */
//  4: PAW32XX_MODE_SCROLL_SNIPE,            /**< High-precision vertical scrolling mode */
//  5: PAW32XX_MODE_SCROLL_HORIZONTAL_SNIPE, /**< High-precision horizontal scrolling mode */
//  6: PAW32XX_MODE_BOTHSCROLL,              /**< XY同時スクロールモード */
  
//LOG_INF("start of get_input_mode_for_current_layer"); // XYSCROLL_DEBUG_LOG

  // Check if using behavior-based switching instead of layer-based
  if (cfg->switch_method != PAW32XX_SWITCH_LAYER) {
    // Convert current mode state to input mode enum
    switch (data->current_mode) {
    case PAW32XX_MODE_SCROLL:
      return PAW32XX_SCROLL;
    case PAW32XX_MODE_SCROLL_HORIZONTAL:
      return PAW32XX_SCROLL_HORIZONTAL;
    case PAW32XX_MODE_SNIPE:
      return PAW32XX_SNIPE;
    case PAW32XX_MODE_SCROLL_SNIPE:
      LOG_INF("switch (data->current_mode)=PAW32XX_SCROLL_SNIPE"); // XYSCROLL_DEBUG_LOG
      return PAW32XX_SCROLL_SNIPE;
    case PAW32XX_MODE_SCROLL_HORIZONTAL_SNIPE:
      return PAW32XX_SCROLL_HORIZONTAL_SNIPE;
    case PAW32XX_MODE_BOTHSCROLL:
      /* XYSCROLL_DEBUG_LOG */
//    LOG_INF("data->current_mode=%d", data->current_mode); // XYSCROLL_DEBUG_LOG
//    LOG_INF("switch (data->current_mode)=PAW32XX_BOTHSCROLL"); // XYSCROLL_DEBUG_LOG
      return PAW32XX_BOTHSCROLL;
    default:
//    LOG_INF("switch (data->current_mode)=PAW32XX_MOVE"); // XYSCROLL_DEBUG_LOG
      return PAW32XX_MOVE;
    }
  }

//LOG_INF("end of switch (data->current_mode)"); // XYSCROLL_DEBUG_LOG
  
  // Original layer-based switching logic
  uint8_t curr_layer = zmk_keymap_highest_layer_active();
/*
  LOG_INF("curr_layer=%d", curr_layer); // XYSCROLL_DEBUG_LOG layer_4がアクティブになっているか
  LOG_INF("cfg->snipe_layers=%d,      cfg->snipe_layers_len=%d",      cfg->snipe_layers,      cfg->snipe_layers_len);       // XYSCROLL_DEBUG_LOG
  LOG_INF("cfg->scroll_layers=%d,     cfg->scroll_layers_len=%d",     cfg->scroll_layers,     cfg->scroll_layers_len);      // XYSCROLL_DEBUG_LOG
  LOG_INF("cfg->bothscroll_layers=%d, cfg->bothscroll_layers_len=%d", cfg->bothscroll_layers, cfg->bothscroll_layers_len);  // XYSCROLL_DEBUG_LOG
*/
  // High-precision horizontal scroll (snipe)
  if (cfg->scroll_horizontal_snipe_layers &&
      cfg->scroll_horizontal_snipe_layers_len > 0) {
    for (size_t i = 0; i < cfg->scroll_horizontal_snipe_layers_len; i++) {
      if (curr_layer == cfg->scroll_horizontal_snipe_layers[i]) {
        return PAW32XX_SCROLL_HORIZONTAL_SNIPE;
      }
    }
  }
  // High-precision vertical scroll (snipe)
  if (cfg->scroll_snipe_layers && cfg->scroll_snipe_layers_len > 0) {
    for (size_t i = 0; i < cfg->scroll_snipe_layers_len; i++) {
      if (curr_layer == cfg->scroll_snipe_layers[i]) {
        return PAW32XX_SCROLL_SNIPE;
      }
    }
  }
  // Horizontal scroll
  if (cfg->scroll_horizontal_layers && cfg->scroll_horizontal_layers_len > 0) {
    for (size_t i = 0; i < cfg->scroll_horizontal_layers_len; i++) {
      if (curr_layer == cfg->scroll_horizontal_layers[i]) {
        return PAW32XX_SCROLL_HORIZONTAL;
      }
    }
  }
  // Vertical scroll
  if (cfg->scroll_layers && cfg->scroll_layers_len > 0) {
    for (size_t i = 0; i < cfg->scroll_layers_len; i++) {
      if (curr_layer == cfg->scroll_layers[i]) {
        return PAW32XX_SCROLL;
      }
    }
  }
  // High-precision cursor movement (snipe)
  if (cfg->snipe_layers && cfg->snipe_layers_len > 0) {
    for (size_t i = 0; i < cfg->snipe_layers_len; i++) {
      if (curr_layer == cfg->snipe_layers[i]) {
        return PAW32XX_SNIPE;
      }
    }
  }
    // XY同時スクロール（BOTHSCROLL）
    if (cfg->bothscroll_layers && cfg->bothscroll_layers_len > 0) {
//    LOG_INF("cfg->bothscroll_layers && cfg->bothscroll_layers_len > 0"); // XYSCROLL_DEBUG_LOG
      for (size_t i = 0; i < cfg->bothscroll_layers_len; i++) {
//      LOG_INF("size_t i = 0; i < cfg->bothscroll_layers_len; i++"); // XYSCROLL_DEBUG_LOG
        if (curr_layer == cfg->bothscroll_layers[i]) {
//        LOG_INF("curr_layer == cfg->bothscroll_layers[i]"); // XYSCROLL_DEBUG_LOG
          return PAW32XX_BOTHSCROLL;
        }
      }
    }
  
//LOG_INF("end of get_input_mode_for_current_layer/ PAW32XX_MOVE"); // XYSCROLL_DEBUG_LOG
  return PAW32XX_MOVE;
}


/**
 * @brief Calculate scroll Y coordinate based on sensor rotation
 *
 * Transforms the raw sensor coordinates to ensure that Y-axis movement
 * always triggers scrolling regardless of the physical sensor orientation.
 * This allows the sensor to be mounted at different angles while maintaining
 * consistent scroll behavior.
 *
 * @param x Raw X coordinate from sensor
 * @param y Raw Y coordinate from sensor  
 * @param rotation Physical sensor rotation in degrees (0, 90, 180, 270)
 * 
 * @return Transformed Y coordinate for scroll calculations
 * 
 * @note For cursor movement, use ZMK input-processors like zip_xy_transform
 *       instead of this function. This is specifically for scroll modes.
 * 
 * @note Handles INT16_MIN overflow case to prevent undefined behavior
 *       when negating the minimum signed integer value.
 */
static int16_t calculate_scroll_y(int16_t x, int16_t y, uint16_t rotation) {
  switch (rotation) {
  case 0:
    return y;
  case 90:
    return x;
  case 180:
    return (y == INT16_MIN) ? INT16_MAX : -y;
  case 270:
    return (x == INT16_MIN) ? INT16_MAX : -x;
  default:
    return y;
  }
}

void paw32xx_motion_timer_handler(struct k_timer *timer) {
  struct paw32xx_data *data =
      CONTAINER_OF(timer, struct paw32xx_data, motion_timer);
  k_work_submit(&data->motion_work);
}

void paw32xx_motion_work_handler(struct k_work *work) {
  struct paw32xx_data *data =
      CONTAINER_OF(work, struct paw32xx_data, motion_work);
  const struct device *dev = data->dev;
  const struct paw32xx_config *cfg = dev->config;
  uint8_t val;
  int16_t x, y;
  int ret;
  bool irq_disabled = true;

  ret = paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
  if (ret < 0) {
    LOG_ERR("Motion register read failed: %d", ret);
    goto cleanup;
  }

  if ((val & MOTION_STATUS_MOTION) == 0x00) {
    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    irq_disabled = false;
    if (gpio_pin_get_dt(&cfg->irq_gpio) == 0) {
      return;
    }
  }

  ret = paw32xx_read_xy(dev, &x, &y);
  if (ret < 0) {
    LOG_ERR("XY data read failed: %d", ret);
    goto cleanup;
  }

  /* reset idle timer on any motion activity */
  /* ensure timeout handler knows which device instance to act on */
  paw32xx_idle_dev = dev;
  /* if we were idle, wake up first */
  if (paw32xx_idle) {
    LOG_INF("PAW32XX: motion detected while idle -> waking up");
    paw32xx_idle_exit(dev);
  }
  /* start/restart the idle timer (timer is initialized at system init) */
  k_timer_start(&paw32xx_idle_timer, K_SECONDS(PAW32XX_IDLE_TIMEOUT_SECONDS), K_NO_WAIT);

  // For scroll modes, we need to transform coordinates based on rotation
  // to ensure y-axis movement always triggers scroll regardless of sensor
  // orientation
  int16_t scroll_y = calculate_scroll_y(x, y, cfg->rotation);

  // Debug log
  LOG_DBG("x=%d y=%d scroll_y=%d rotation=%d", x, y, scroll_y, cfg->rotation);

  enum paw32xx_input_mode input_mode = get_input_mode_for_current_layer(dev);

  // CPI Switching
  int16_t target_cpi = cfg->res_cpi;
  if (input_mode == PAW32XX_SNIPE) {
    // Use snipe_cpi if configured, otherwise use default from Kconfig
    target_cpi =
        (cfg->snipe_cpi > 0) ? cfg->snipe_cpi : CONFIG_PAW3222_SNIPE_CPI;
  }
  if (data->current_cpi != target_cpi) {
    ret = paw32xx_set_resolution(dev, target_cpi);
    if (ret == 0) {
      data->current_cpi = target_cpi;
    } else {
      LOG_WRN("Failed to set CPI to %d: %d", target_cpi, ret);
    }
  }
  

  /* XYSCROLL_DEBUG_LOG */
  LOG_INF("input_mode=%d", input_mode); // XYSCROLL_DEBUG_LOG

  switch (input_mode) {
  case PAW32XX_MOVE: { // Normal cursor movement
    input_report_rel(data->dev, INPUT_REL_X, x, false, K_NO_WAIT);
    input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);
    break;
  }
  case PAW32XX_SNIPE: { // High-precision cursor movement
    uint8_t divisor = MAX(1, cfg->snipe_divisor);
    int16_t snipe_x = x / divisor;
    int16_t snipe_y = y / divisor;
    input_report_rel(data->dev, INPUT_REL_X, snipe_x, false, K_NO_WAIT);
    input_report_rel(data->dev, INPUT_REL_Y, snipe_y, true, K_FOREVER);
    break;
  }
  case PAW32XX_SCROLL: // Vertical scroll
    process_scroll_input(data->dev, &data->scroll_accumulator, scroll_y, cfg->scroll_tick, false);
    break;
  case PAW32XX_SCROLL_HORIZONTAL: // Horizontal scroll
    process_scroll_input(data->dev, &data->scroll_accumulator, scroll_y, cfg->scroll_tick, true);
    break;
  case PAW32XX_SCROLL_SNIPE: // High-precision vertical scroll
    {
      uint8_t divisor = MAX(1, cfg->scroll_snipe_divisor);
      int16_t snipe_scroll_y = scroll_y / divisor;
      process_scroll_input(data->dev, &data->scroll_accumulator, snipe_scroll_y, cfg->scroll_snipe_tick, false);
    }
    break;
  case PAW32XX_SCROLL_HORIZONTAL_SNIPE: // High-precision horizontal scroll
    {
      uint8_t divisor = MAX(1, cfg->scroll_snipe_divisor);
      int16_t snipe_scroll_y = scroll_y / divisor;
      process_scroll_input(data->dev, &data->scroll_accumulator, snipe_scroll_y, cfg->scroll_snipe_tick, true);
    }
    break;
  case PAW32XX_BOTHSCROLL: // XY同時スクロール
    {
      // X軸スクロール値の算出（必要に応じて座標変換）
      int16_t scroll_x = calculate_scroll_y(y, x, cfg->rotation); // X/Y入れ替えでX軸用
      process_scroll_input(data->dev, &data->scroll_accumulator_x, scroll_x, cfg->scroll_tick, true);
      process_scroll_input(data->dev, &data->scroll_accumulator_y, scroll_y, cfg->scroll_tick, false);

    }
    break;
  default:
    LOG_ERR("Unknown input_mode: %d", input_mode);
    break;
  }

  k_timer_start(&data->motion_timer, K_MSEC(15), K_NO_WAIT);
  return;

cleanup:
  if (irq_disabled) {
    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
  }
}

void paw32xx_motion_handler(const struct device *gpio_dev,
                            struct gpio_callback *cb, uint32_t pins) {
  ARG_UNUSED(gpio_dev);
  ARG_UNUSED(pins);
  struct paw32xx_data *data = CONTAINER_OF(cb, struct paw32xx_data, motion_cb);
  const struct device *dev = data->dev;
  const struct paw32xx_config *cfg = dev->config;

  gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
  k_timer_stop(&data->motion_timer);
  /* If idle, wake up sensor and resume processing */
  if (paw32xx_idle) {
    LOG_INF("PAW32XX: IRQ while idle -> waking up");
    paw32xx_idle_exit(dev);
    return;
  }
  k_work_submit(&data->motion_work);
}

void paw32xx_idle_timeout_handler(struct k_timer *timer) {
  const struct device *dev = paw32xx_idle_dev;
  struct paw32xx_data *data = dev->data;
  const struct paw32xx_config *cfg = dev->config;

  LOG_INF("PAW32XX: idle timeout reached, entering idle");

  /* disable irq */
  gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);

  /* cancel motion processing */
  k_work_cancel(&data->motion_work);
  k_timer_stop(&data->motion_timer);

  /* attempt to put sensor to low-power if available */
#ifdef CONFIG_PAW3222_POWER_CTRL
#if defined(CONFIG_PAW3222_POWER_CTRL)
  int rc_sleep = paw3222_set_sleep(dev, true);
  if (rc_sleep) {
    LOG_WRN("PAW32XX: paw3222_set_sleep(true) failed: %d", rc_sleep);
  } else {
    LOG_INF("PAW32XX: sensor set to sleep");
  }
#else
  (void)paw3222_set_sleep(dev, true);
#endif
#endif

  paw32xx_idle = true;
}

void paw32xx_idle_enter(const struct device *dev) {
  /* wrapper if needed in future */
  (void)dev;
}

void paw32xx_idle_exit(const struct device *dev) {
  struct paw32xx_data *data = dev->data;
  const struct paw32xx_config *cfg = dev->config;
  int rc;

  if (!paw32xx_idle) {
    return;
  }

#ifdef CONFIG_PAW3222_POWER_CTRL
  {
    int rc = paw3222_set_sleep(dev, false);
    if (rc) {
      LOG_WRN("PAW32XX: paw3222_set_sleep(false) failed: %d", rc);
    } else {
      LOG_INF("PAW32XX: sensor wake request succeeded");
    }
  }
#endif

  /* re-enable irq */
  gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);

  /* restart motion processing */
  k_timer_start(&data->motion_timer, K_MSEC(15), K_NO_WAIT);
  k_work_submit(&data->motion_work);

  paw32xx_idle = false;
  /* restart idle timer */
  k_timer_start(&paw32xx_idle_timer, K_SECONDS(PAW32XX_IDLE_TIMEOUT_SECONDS), K_NO_WAIT);
  LOG_INF("PAW32XX: exited idle and resumed normal operation");
}

static int paw32xx_idle_init(void) {
  if (!paw32xx_idle_timer_inited) {
    k_timer_init(&paw32xx_idle_timer, paw32xx_idle_timeout_handler, NULL);
    paw32xx_idle_timer_inited = true;
  }
  return 0;
}

SYS_INIT(paw32xx_idle_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);