/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Matthias P. Braendli
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "config.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "common.hpp"
#include "pins.hpp"
#include "relays.hpp"
#include "ltc2400.h"

extern "C" {
#include "uart.h"
}

// UART endline is usually CR LF
#define ENDL "\r\n"

constexpr double R_SHUNT = 1e-3; // Ohm

/* Capacity counters and thresholds, in As (= Coulombs)
 *
 * For every relay, define a threshold below which the
 * relay should be active.
 */
constexpr double THRESHOLD_K1 = 1200.0 * 3600;
constexpr double THRESHOLD_K2 = 1000.0 * 3600;
constexpr double THRESHOLD_K3 = 600.0 * 3600;
constexpr double THRESHOLD_HYSTERESIS = 10.0 * 3600;

constexpr double THRESHOLD_K1_UP = 1200.0 * 3600 + THRESHOLD_HYSTERESIS;
constexpr double THRESHOLD_K2_UP = 1000.0 * 3600 + THRESHOLD_HYSTERESIS;
constexpr double THRESHOLD_K3_UP = 600.0 * 3600 + THRESHOLD_HYSTERESIS;

constexpr double THRESHOLD_K1_DOWN = 1200.0 * 3600 - THRESHOLD_HYSTERESIS;
constexpr double THRESHOLD_K2_DOWN = 1000.0 * 3600 - THRESHOLD_HYSTERESIS;
constexpr double THRESHOLD_K3_DOWN = 600.0 * 3600 - THRESHOLD_HYSTERESIS;

constexpr uint32_t MAX_CAPACITY = 1500uL * 3600uL; // As

static uint32_t current_capacity;
static uint32_t previous_capacity;
static bool relay_state_known = false;

/* Storage of battery capacity in mC.
 * 3600 mC = 1mAh */

/* Store the capacity three times in EEPROM, and check data validity using majority vote */
uint32_t EEMEM stored_capacity1;
uint32_t EEMEM stored_capacity2;
uint32_t EEMEM stored_capacity3;

/* Store timestamp of previous execution of a subroutine.
 * Subroutines scheduled in second intervals don't need to
 * be stored as timer_t, to save some space and simplify
 * the comparison.
 */
static uint32_t last_store_time_seconds;
static uint32_t last_threshold_calculation_seconds;
static uint32_t last_ltc2400_print_time_seconds;
static timer_t last_ltc2400_measure;

enum class adc_state_t {
    IDLE,
    PENDING_ADC0,
    PENDING_ADC1,
};

static adc_state_t adc_state;
static uint32_t last_adc_measure_time_seconds;

/* Raw values from the ADC.
 * The ADC converts an analog input voltage to a 10-bit digital value through
 * successive approximation. The minimum value represents GND and the maximum
 * value represents the voltage on the AREF pin minus 1 LSB.
 * (datasheet 24.2)
 */
const uint32_t V_REF_mV = 5000;

#define ADC_VALUE_TO_MILLIVOLT(val) ((uint32_t)val * V_REF_mV) / (uint32_t)(1<<10)

// Use the LDO on Vref as ADC reference, set REFS1..REFS0 = 0b00, and ADC input 0
#define SET_ADMUX(input) ADMUX = (input & 0x0F)

/* Timer at approximately 100ms.
 *
 * Setup 100Hz timer, assuming F_CPU at 16MHz / 8:
 *
 * overflow for 100ms: F_CPU [ticks/s] / prescaler [unitless] * interval [s] = [ticks/s*s] = [ticks]
 * interval [s] = 0.1 = 1 / 10
 *
 * Actual interval after rounding:
 * interval [s] = overflow [ticks] / (F_CPU [ticks/s] / prescaler [unit-less])
 *              = 99.84 ms
 */
constexpr uint8_t TIMER_OVERFLOW = (uint8_t)(F_CPU / 1024 / 10);
constexpr double TIMER_TICK_INTERVAL = (double)TIMER_OVERFLOW / ((double)F_CPU / 1024.0); // == 0.099840 s
constexpr uint32_t TIMER_TICK_INTERVAL_US = (uint32_t)(TIMER_TICK_INTERVAL * 1000000.0);

/* Since this timer is updated in an ISR, care has to be taken
 * when reading it, because all operations involving variables
 * larger than 1 byte are not atomic on AVR.
 */
static timer_t system_timer;

/* At reset, save the mcusr register to find out why we got reset.
 * Datasheet 11.9.1, example code from wdt.h */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

// Convert from As to mAh
inline uint32_t As_to_mAh(uint32_t value)
{
    float cap = value;
    cap /= 3600.0f;
    cap *= 1000.0f;
    return lrintf(cap);
}

ISR(TIMER0_OVF_vect)
{
    system_timer += timer_t{0, TIMER_TICK_INTERVAL_US};
}

enum class error_type_t {
    EEPROM_READ_WARNING,
    EEPROM_READ_ERROR,
    EEPROM_WRITE_ERROR,
    LTC2400_NOT_READY,
    LTC2400_DMY_BIT_FAULT,
    LTC2400_EXTENDED_RANGE_ERROR,
    RELAY_NOT_SET,
};

static void flag_error(const error_type_t e);

static char timestamp_buf[32];
static void send_message(const char *message)
{
    snprintf(timestamp_buf, sizeof(timestamp_buf), "TEXT,%ld,", system_timer.get_seconds_atomic());
    uart_puts(timestamp_buf);
    uart_puts(message);
    uart_puts_P(ENDL);
}

#define send_debug(x) send_message(x)

static void send_capacity(uint32_t capacity, const timer_t& time)
{
    snprintf(timestamp_buf, sizeof(timestamp_buf), "CAPA,%ld,%ld" ENDL, time.get_seconds_atomic(), capacity);
    uart_puts(timestamp_buf);
}

enum class adc_voltage_id {
    V_BAT_PLUS,
    V_BAT_MINUS,
};

static void send_voltage(uint32_t millivolts, adc_voltage_id voltage_id, const timer_t& time)
{
    snprintf(timestamp_buf, sizeof(timestamp_buf), "VBAT%c,%ld,%ld" ENDL,
            voltage_id == adc_voltage_id::V_BAT_PLUS ? '+' : '-',
            time.get_seconds_atomic(), millivolts);
    uart_puts(timestamp_buf);
}


static void load_capacity_from_eeprom()
{
    /* Store the same value three times to make
     * a majority vote to detect errors
     */
    uint32_t cap1 = eeprom_read_dword(&stored_capacity1);
    uint32_t cap2 = eeprom_read_dword(&stored_capacity2);
    uint32_t cap3 = eeprom_read_dword(&stored_capacity3);

    if (cap1 == cap2 and cap2 == cap3) {
        current_capacity = cap1;
    }
    else if (cap1 == cap2) {
        flag_error(error_type_t::EEPROM_READ_WARNING);
        current_capacity = cap1;
        eeprom_write_dword(&stored_capacity3, cap1);
    }
    else if (cap1 == cap3) {
        flag_error(error_type_t::EEPROM_READ_WARNING);
        current_capacity = cap1;
        eeprom_write_dword(&stored_capacity2, cap1);
    }
    else if (cap2 == cap3) {
        flag_error(error_type_t::EEPROM_READ_WARNING);
        current_capacity = cap2;
        eeprom_write_dword(&stored_capacity1, cap1);
    }
    else {
        flag_error(error_type_t::EEPROM_READ_ERROR);
        current_capacity = cap2; // arbitrary
    }

    if (current_capacity == 0xFFffFFff) {
        /* EEPROM does not contain a valid value */
        current_capacity = MAX_CAPACITY;
    }

    previous_capacity = current_capacity;
}

static void store_capacity_to_eeprom()
{
    eeprom_write_dword(&stored_capacity1, current_capacity);
    eeprom_write_dword(&stored_capacity2, current_capacity);
    eeprom_write_dword(&stored_capacity3, current_capacity);

    if (eeprom_read_dword(&stored_capacity1) != current_capacity or
        eeprom_read_dword(&stored_capacity2) != current_capacity or
        eeprom_read_dword(&stored_capacity3) != current_capacity) {
        flag_error(error_type_t::EEPROM_WRITE_ERROR);
    }
}

static void handle_thresholds(const timer_t& time_now)
{
    if (not relay_state_known) {
        /* At bootup, ignore hysteresis. Put all relays in the state defined by the
         * thresholds.
         */
        bool success = relays_toggle(relay_id_t::K1, current_capacity < THRESHOLD_K1, time_now);
        success &= relays_toggle(relay_id_t::K2, current_capacity < THRESHOLD_K2, time_now);
        success &= relays_toggle(relay_id_t::K3, current_capacity < THRESHOLD_K3, time_now);
        relay_state_known = success;

        if (not success) {
            flag_error(error_type_t::RELAY_NOT_SET);
        }
    }
    else {
        bool success = true;

        if (previous_capacity < THRESHOLD_K1_UP and current_capacity >= THRESHOLD_K1_UP) {
            send_message("K1 reset");
            success &= relays_toggle(relay_id_t::K1, false, time_now);
        }

        if (previous_capacity > THRESHOLD_K1_DOWN and current_capacity <= THRESHOLD_K1_DOWN) {
            send_message("K1 set");
            success &= relays_toggle(relay_id_t::K1, true, time_now);
        }

        if (previous_capacity < THRESHOLD_K2_UP and current_capacity >= THRESHOLD_K2_UP) {
            send_message("K2 reset");
            success &= relays_toggle(relay_id_t::K2, false, time_now);
        }

        if (previous_capacity > THRESHOLD_K2_DOWN and current_capacity <= THRESHOLD_K2_DOWN) {
            send_message("K2 set");
            success &= relays_toggle(relay_id_t::K2, true, time_now);
        }

        if (previous_capacity < THRESHOLD_K3_UP and current_capacity >= THRESHOLD_K3_UP) {
            send_message("K3 reset");
            success &= relays_toggle(relay_id_t::K3, false, time_now);
        }

        if (previous_capacity > THRESHOLD_K3_DOWN and current_capacity <= THRESHOLD_K3_DOWN) {
            send_message("K3 set");
            success &= relays_toggle(relay_id_t::K3, true, time_now);
        }

        if (not success) {
            flag_error(error_type_t::RELAY_NOT_SET);
        }
    }

    previous_capacity = current_capacity;
}

static void flag_error(const error_type_t e)
{
    snprintf(timestamp_buf, sizeof(timestamp_buf), "ERROR,%ld,", system_timer.get_seconds_atomic());
    uart_puts(timestamp_buf);
    switch (e) {
        case error_type_t::EEPROM_READ_WARNING:
            uart_puts_P("EEPRON read warning" ENDL);
            break;
        case error_type_t::EEPROM_READ_ERROR:
            uart_puts_P("EEPRON read error" ENDL);
            break;
        case error_type_t::EEPROM_WRITE_ERROR:
            uart_puts_P("EEPRON write error" ENDL);
            break;
        case error_type_t::LTC2400_NOT_READY:
            uart_puts_P("LTC2400 not ready" ENDL);
            break;
        case error_type_t::LTC2400_DMY_BIT_FAULT:
            uart_puts_P("LTC2400 DMY bit error" ENDL);
            break;
        case error_type_t::LTC2400_EXTENDED_RANGE_ERROR:
            uart_puts_P("LTC2400 extended range error" ENDL);
            break;
        case error_type_t::RELAY_NOT_SET:
            uart_puts_P("RELAYS not set" ENDL);
            break;
    }
}

int main()
{
    /* Save the reset source for debugging, then enable the watchdog.
     * Attention: WDT may be already enabled if it triggered a reset!
     * Datasheet 11.8.2 */
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_reset();
    wdt_enable(WDTO_4S);

    /* Setup GPIO */
    // Active-low outputs must be high
    // PINB_SPI_SCK must be low (See ltc2400.h)
    PORTB = PINB_INIT;
    PORTC = PINC_INIT;
    PORTD = PIND_INIT;

    // Enable output
    DDRB = PINB_OUTPUTS;
    DDRC = PINC_OUTPUTS;
    DDRD = PIND_OUTPUTS;

    pins_set_status(true);

    relays_init();

    // Initialise SPI and LTC2400
    ltc2400_init();

    // Enable ADC
    ADCSRA |= _BV(ADEN);
    SET_ADMUX(0);

    // Warning: Bi-stable relays are still in unknown state!

    /* Setup UART */
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    if (mcusr_mirror & WDRF) {
        send_message("Startup after WDT reset");
    }
    else if (mcusr_mirror & BORF) {
        send_message("Startup after brown-out");
    }
    else if (mcusr_mirror & EXTRF) {
        send_message("Startup after external reset");
    }
    else if (mcusr_mirror & PORF) {
        send_message("Startup after power-on reset");
    }
    else {
        send_message("Startup");
    }

    system_timer = timer_t(0, 0);

    /* Load capacity stored in EEPROM */
    current_capacity = 0;
    load_capacity_from_eeprom();

    snprintf(timestamp_buf, sizeof(timestamp_buf), "TEXT,%d,Load EEPROM %lu" ENDL, 0, As_to_mAh(current_capacity));
    uart_puts(timestamp_buf);

    last_ltc2400_measure = system_timer;

    last_store_time_seconds =
        last_ltc2400_print_time_seconds =
        last_adc_measure_time_seconds =
        last_threshold_calculation_seconds = system_timer.get_seconds_atomic();

    adc_state = adc_state_t::IDLE;

    /* Enable timer and interrupts */
    TCCR0B |= _BV(WGM02); // Set timer mode to CTC (datasheet 15.7.2)
    TIMSK0 |= _BV(TOIE0); // enable overflow interrupt
    OCR0A = TIMER_OVERFLOW;
    TCCR0B |= _BV(CS02) | _BV(CS00); // Start timer at Fcpu/1024
    sei();
    send_debug("Timer started");

    // Accumulate in floating point
    double capacity_accum = current_capacity;

    /* Put the CPU to sleep */
    set_sleep_mode(SLEEP_MODE_IDLE);

    send_debug("Sleep configured");
    while (true) {
        pins_set_status(false);
        sleep_mode();
        wdt_reset();
        pins_set_status(true);

        /* In every loop, access the system_timer only once, so that
         * every loop has a well-defined time */
        const auto time_now = system_timer.get_atomic_copy();

#if ENABLE_STORE_TO_EEPROM
        /* EEPROM has an endurance of at least 100'000 write/erase cycles.
         * (Datasheet 8.4 EEPROM Data Memory)
         * Storing every five hours gives us several years of endurance.
         * */
        if (last_store_time_seconds + 3600 * 5 <= time_now.seconds_) {
            send_debug("Store to EEPROM");
            store_capacity_to_eeprom();
        }
#endif
        const auto ltc2400_measure_interval = timer_t{0, 200000uL};
        if (last_ltc2400_measure + ltc2400_measure_interval < time_now) {
            last_ltc2400_measure += ltc2400_measure_interval;

            if (ltc2400_conversion_ready()) {
                bool dmy_fault = false;
                bool exr_fault = false;
                uint32_t adc_value = 0;
                const double adc_voltage = ltc2400_get_conversion_result(dmy_fault, exr_fault, adc_value);

                if (dmy_fault) {
                    flag_error(error_type_t::LTC2400_DMY_BIT_FAULT);
                }

                if (exr_fault) {
                    flag_error(error_type_t::LTC2400_EXTENDED_RANGE_ERROR);
                }

                /* Vout - 2.5V = Ishunt * Rshunt * 20 */
                const double i_shunt = (adc_voltage - 2.5) / (20.0 * R_SHUNT);
                capacity_accum += i_shunt * ltc2400_measure_interval.microsecs_ * 1e-6;

#if DEBUG_MEASUREMENT
                snprintf(timestamp_buf, sizeof(timestamp_buf), "DBG,%ldmV,%ldmA" ENDL,
                        lrint((adc_voltage - 2.5) * 1e3),
                        lrint(i_shunt * 1e3));
                uart_puts(timestamp_buf);
#endif

                if (capacity_accum < 0) { capacity_accum = 0; }
                if (capacity_accum > MAX_CAPACITY) { capacity_accum = MAX_CAPACITY; }

                current_capacity = lrint(capacity_accum);
            }
            else {
                flag_error(error_type_t::LTC2400_NOT_READY);
            }
        }

        constexpr auto threshold_calculation_interval_s = 4;
        if (last_threshold_calculation_seconds + threshold_calculation_interval_s < time_now.seconds_) {
            last_threshold_calculation_seconds += threshold_calculation_interval_s;
            handle_thresholds(time_now);
        }

        constexpr auto ltc2400_print_interval_s = 10;
        if (last_ltc2400_print_time_seconds + ltc2400_print_interval_s < time_now.seconds_) {
            last_ltc2400_print_time_seconds += ltc2400_print_interval_s;
            send_capacity(As_to_mAh(current_capacity), time_now);
        }

        // Input is divided by 4 by LM324. ADC is 10-bit,
        // value 0 is GND, value (1<<10) is Vref
        constexpr auto adc_interval_s = 20;
        switch (adc_state) {
            case adc_state_t::IDLE:
                if (last_adc_measure_time_seconds + adc_interval_s < time_now.seconds_) {
                    last_adc_measure_time_seconds += adc_interval_s;
                    SET_ADMUX(0);
                    // Start ADC conversion
                    ADCSRA |= _BV(ADSC);
                    adc_state = adc_state_t::PENDING_ADC0;
                }
                break;
            case adc_state_t::PENDING_ADC0:
                // ADSC is cleared when the conversion finishes
                if ((ADCSRA & ADSC) == 0) {
                    // BAT+

                    // Datasheet 24.9.3 says ADCL must be read first
                    const uint8_t adcl = ADCL;
                    const uint16_t adc_value_0 = ((uint16_t)ADCH << 8) | adcl;
                    send_voltage(ADC_VALUE_TO_MILLIVOLT(adc_value_0) * 4, adc_voltage_id::V_BAT_PLUS, time_now);
                    SET_ADMUX(1);
                    // Start ADC conversion
                    ADCSRA |= _BV(ADSC);

                    adc_state = adc_state_t::PENDING_ADC1;
                }
                break;
            case adc_state_t::PENDING_ADC1:
                // ADSC is cleared when the conversion finishes
                if ((ADCSRA & ADSC) == 0) {
                    // BAT-
                    const uint8_t adcl = ADCL;
                    const uint16_t adc_value_1 = ((uint16_t)ADCH << 8) | adcl;
                    adc_state = adc_state_t::IDLE;

                    send_voltage(ADC_VALUE_TO_MILLIVOLT(adc_value_1) * 4, adc_voltage_id::V_BAT_MINUS, time_now);
                }
                break;
        }

        relays_handle(time_now);
    }

    return 0;
}
