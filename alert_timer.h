#ifndef ALERT_TIMER_H__
#define ALERT_TIMER_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALERT_TIMER_INTERVAL 100 //10.0 Sec
#define ALERT_LED 1

void alert_timer_timeout_handler(void *p_context);
void alert_sound();

#endif //ALERT_TIMER_H__
