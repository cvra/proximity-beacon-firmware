#include <ch.h>
#include <hal.h>
#include <stdbool.h>
#include <math.h>
#include "timestamp/timestamp.h"
#include "rpm.h"
#include "control.h"
#include "proximity_beacon.h"

// max 7 signals per tour
#define SIGNAL_BUFFER_SIZE  2*7
#define MAX_SPEED           50*2*M_PI

struct proximity_beacon_signal proximity_beacon_array[SIGNAL_BUFFER_SIZE];
msg_t proximity_beacon_buffer[SIGNAL_BUFFER_SIZE];
mailbox_t proximity_beacon_mbox;
memory_pool_t proximity_beacon_pool;

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel);

static const EXTConfig extcfg = {{
    {EXT_CH_MODE_DISABLED, NULL}, // 0
    {EXT_CH_MODE_DISABLED, NULL}, // 1
    {EXT_CH_MODE_DISABLED, NULL}, // 2
    {EXT_CH_MODE_DISABLED, NULL}, // 3
    // hal sensor, PB4
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback},
    {EXT_CH_MODE_DISABLED, NULL}, // 5
    {EXT_CH_MODE_DISABLED, NULL}, // 6
    {EXT_CH_MODE_DISABLED, NULL}, // 7
    {EXT_CH_MODE_DISABLED, NULL}, // 8
    {EXT_CH_MODE_DISABLED, NULL}, // 9
    {EXT_CH_MODE_DISABLED, NULL}, // 10
    {EXT_CH_MODE_DISABLED, NULL}, // 11
    // SICK active low, PA12
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, gpio_exti_callback},
    {EXT_CH_MODE_DISABLED, NULL}, // 13
    {EXT_CH_MODE_DISABLED, NULL}, // 14
    {EXT_CH_MODE_DISABLED, NULL}, // 15
    {EXT_CH_MODE_DISABLED, NULL}, // 16
    {EXT_CH_MODE_DISABLED, NULL}, // 17
    {EXT_CH_MODE_DISABLED, NULL}, // 18
    {EXT_CH_MODE_DISABLED, NULL}, // 19
    {EXT_CH_MODE_DISABLED, NULL}, // 20
    {EXT_CH_MODE_DISABLED, NULL}, // 21
    {EXT_CH_MODE_DISABLED, NULL}  // 22
}};

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    static float start_angle;
    static bool signal_active;
    if (channel == GPIOA_GPIO_I) {
        if (!palReadPad(GPIOA, GPIOA_GPIO_I)) {
            // light signal received
            start_angle = rpm_get_position();
            signal_active = true;
        } else {
            // light signal lost
            if (signal_active) {
                float length = rpm_get_position() - start_angle;
                signal_active = false;
                chSysLockFromISR();
                struct proximity_beacon_signal *sp;
                sp = chPoolAllocI(&proximity_beacon_pool);
                if (sp) {
                    sp->start_angle = start_angle;
                    sp->length = length;
                    chMBPostI(&proximity_beacon_mbox, (msg_t)sp);
                }
                chSysUnlockFromISR();
            }
        }
    } else if (channel == GPIOB_GPIO_A) { // hal sensor
        rpm_barrier_crossing(timestamp_get());
    }
}

void proximity_beacon_init(void)
{
    chMBObjectInit(&proximity_beacon_mbox, proximity_beacon_buffer, SIGNAL_BUFFER_SIZE);
    chPoolObjectInit(&proximity_beacon_pool, sizeof(struct proximity_beacon_signal), NULL);
    chPoolLoadArray(&proximity_beacon_pool, proximity_beacon_array, SIGNAL_BUFFER_SIZE);
    extStart(&EXTD1, &extcfg);
}

struct proximity_beacon_signal *proximity_beacon_signal_get(void)
{
    struct proximity_beacon_signal *sp;
    msg_t m = chMBFetch(&proximity_beacon_mbox, (msg_t *)&sp, TIME_IMMEDIATE);
    if (m != MSG_OK) {
        return NULL;
    }
    return sp;
}

void proximity_beacon_signal_delete(struct proximity_beacon_signal *sp)
{
    chPoolFree(&proximity_beacon_pool, sp);
}

void proximity_beacon_set_speed(float omega)
{
    if (omega > MAX_SPEED) {
        omega = MAX_SPEED;
    }
    control_update_velocity_setpoint(omega);
}
