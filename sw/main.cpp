/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Matthias P. Braendli
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

#include "pins.h"
#include "ltc2400.h"

extern "C" {
#include "uart.h"
}

// UART endline is usually CR LF
#define ENDL "\r\n"

constexpr double R_SHUNT = 5e-3;
constexpr int TICKS_PER_SECOND = 10;

struct timer_t {
    uint32_t seconds = 0; /* Timer in seconds */
    uint8_t ticks = 0; /* Timer in 100ms steps */

    timer_t() {}
    timer_t(uint32_t seconds_, uint8_t ticks_) : seconds(seconds_), ticks(ticks_) {}

    bool operator>(const timer_t& rhs) {
        return (seconds > rhs.seconds) or
            (seconds == rhs.seconds and ticks > rhs.ticks);
    }

    void normalise() {
        while (ticks >= 10) {
            seconds++;
            ticks -= 10;
        }
    }

    timer_t operator+(const timer_t& rhs) {
        timer_t t;
        t.seconds = seconds + rhs.seconds;
        t.ticks = ticks + rhs.ticks;
        t.normalise();
        return t;
    }

    timer_t operator+(uint8_t ticks) {
        timer_t t = timer_t(0, ticks);
        return *this + t;
    }

    void operator+=(const timer_t& inc) {
        seconds += inc.seconds;
        ticks += inc.ticks;
        normalise();
    }

    void operator+=(uint8_t ticks) {
        *this += timer_t(0, ticks);
    }

    static constexpr int ms_to_ticks(int ms) { return ms / 100; }
    static timer_t from_seconds(uint32_t s) { return timer_t(s, 0); }
};

/* Storage of battery capacity in mC.
 * 3600 mC = 1mAh */

/* Store the capacity three times in EEPROM, and check data validity using majority vote */
uint32_t EEMEM stored_capacity1;
uint32_t EEMEM stored_capacity2;
uint32_t EEMEM stored_capacity3;
uint32_t last_store_time; /* In seconds */

timer_t last_ltc2400_measure;
timer_t last_ltc2400_print_time;

uint32_t current_capacity;

/* Timer at approximately 100ms */
static timer_t system_timer;

/* At reset, save the mcusr register to find out why we got reset.
 * Datasheet 11.9.1, example code from wdt.h */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

ISR(TIMER0_COMPA_vect)
{
    if (system_timer.ticks == 9) {
        system_timer.seconds++;
        system_timer.ticks = 0;
    }
    else {
        system_timer.ticks++;
    }
}

enum class error_type_t {
    EEPROM_READ_WARNING,
    EEPROM_READ_ERROR,
    EEPROM_WRITE_ERROR,
    LTC2400_DMY_BIT_FAULT,
    LTC2400_EXTENDED_RANGE_ERROR,
};

static void flag_error(const error_type_t e);

static void load_capacity_from_eeprom()
{
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

static char timestamp_buf[16];
static void send_message(const char *message)
{
    snprintf(timestamp_buf, 15, "TEXT,%ld,", system_timer.seconds);
    uart_puts(timestamp_buf);
    uart_puts(message);
    uart_puts_P(ENDL);
}

static void send_capacity(uint32_t capacity)
{
    snprintf(timestamp_buf, 15, "CAPACITY,%ld,%ld" ENDL, system_timer.seconds, capacity);
    uart_puts(timestamp_buf);
}

static void flag_error(const error_type_t e)
{
    snprintf(timestamp_buf, 15, "ERROR,%ld,", system_timer.seconds);
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
        case error_type_t::LTC2400_DMY_BIT_FAULT:
            uart_puts_P("LTC2400 DMY bit error" ENDL);
            break;
        case error_type_t::LTC2400_EXTENDED_RANGE_ERROR:
            uart_puts_P("LTC2400 extended range error" ENDL);
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

    current_capacity = 0;
#warning "Initialise current_capacity properly"

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

    // Initialise SPI and LTC2400
    ltc2400_init();

    // Use the LDO on Vref as ADC reference, set REFS1..REFS0 = 0b00
    ADMUX &= ~(_BV(REFS0) | _BV(REFS1));

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

    /* Setup 100Hz timer, assuming F_CPU at 16MHz / 8:
     *
     * overflow for 100ms: F_CPU [ticks/s] / prescaler [unit-less] * interval [s] = [ticks/s*s] = [ticks]
     * interval [s] = 0.1 = 1 / 10
     *
     * Actual interval after rounding:
     * interval [s] = overflow [ticks] / (F_CPU [ticks/s] / prescaler [unit-less])
     *              = 99.84 ms
     */
    system_timer.seconds = 0;
    system_timer.ticks = 0;
    TCCR0B |= _BV(WGM02); // Set timer mode to CTC (datasheet 15.7.2)
    TIMSK0 |= _BV(TOIE0); // enable overflow interrupt
    OCR0A = (uint8_t)(F_CPU / 1024 / 10); // Overflow at 99.84 ms
    TCCR0B |= _BV(CS02) | _BV(CS00); // Start timer at Fcpu/1024

    /* Load capacity stored in EEPROM */
    load_capacity_from_eeprom();
    last_ltc2400_print_time = last_ltc2400_measure = system_timer;
    last_store_time = system_timer.seconds;

    /* Enable interrupts */
    sei();

#warning "Decode if we should accumulate in a double or uint32 */
    double accum = current_capacity;

    /* Put the CPU to sleep */
    set_sleep_mode(SLEEP_MODE_IDLE);
    while (true) {
        sleep_mode();

        pins_set_status(system_timer.ticks == 0);

        if (last_store_time + 3600 * 5 >= system_timer.seconds) {
            store_capacity_to_eeprom();
        }

        constexpr auto ltc2400_measure_interval = timer_t::ms_to_ticks(100);
        if (last_ltc2400_measure + ltc2400_measure_interval > system_timer) {
            last_ltc2400_measure += ltc2400_measure_interval;

            if (ltc2400_conversion_ready()) {
                bool dmy_fault = false;
                bool exr_fault = false;
                const float adc_voltage = ltc2400_get_conversion_result(dmy_fault, exr_fault);

                if (dmy_fault) {
                    flag_error(error_type_t::LTC2400_DMY_BIT_FAULT);
                }

                if (exr_fault) {
                    flag_error(error_type_t::LTC2400_EXTENDED_RANGE_ERROR);
                }

                /* Vout - 2.5V = Ishunt * Rshunt * 20 */
                const double i_shunt = (adc_voltage - 2.5) / (20.0 * R_SHUNT);
                accum += i_shunt / TICKS_PER_SECOND;
                current_capacity = lrint(accum);
            }
        }

        const auto ltc2400_print_interval = timer_t::from_seconds(10);
        if (last_ltc2400_print_time + ltc2400_print_interval > system_timer) {
            last_ltc2400_print_time += ltc2400_print_interval;
            send_capacity(current_capacity);
        }
    }

    return 0;
}
