#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_soc.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "bsp.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_error.h"
#include "nordic_common.h"
#include "alert_timer.h"

static int alert_state = 0;
static int alert_time = 0;

void alert_timer_timeout_handler(void *p_context) {
    if (alert_state != 0) {
        // Sound alert
        bsp_board_led_on(ALERT_LED);
        nrf_gpio_pin_clear(31);
        if (alert_time > 0) {
            alert_time --;
        } else {
            bsp_board_led_off(ALERT_LED);
            alert_state = 0;
            nrf_gpio_pin_set(31);
        }
    }
    return;
}

void alert_sound() {
  alert_state = 1;
  alert_time = 5000;
}
