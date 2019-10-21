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

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

extern "C" {
#include "uart.h"
}

// Definitions allocation pin
#define PIN(x) (1 << x)

// UART endline is usually CR LF
#define ENDL "\r\n"

/* All relay signals PINx_Kx have external pulldown.
 * All pins whose name ends in 'n' are active low */

// PORT B
constexpr uint8_t PINB_STATUSn = PIN(0);
// Pins 2,3,4,5 = SPI
constexpr uint8_t PINB_SPI_LTC_CSn = PIN(2); // with external pullup
constexpr uint8_t PINB_SPI_MOSI = PIN(3);
constexpr uint8_t PINB_SPI_MISO = PIN(4);
constexpr uint8_t PINB_SPI_SCK = PIN(5);

constexpr uint8_t PINB_OUTPUTS =
    PINB_STATUSn | PINB_SPI_SCK | PINB_SPI_MOSI | PINB_SPI_LTC_CSn;

// PORT C
constexpr uint8_t PINC_ADC0 = PIN(0);
constexpr uint8_t PINC_ADC1 = PIN(1);
constexpr uint8_t PINC_K3_RESET = PIN(2);
constexpr uint8_t PINC_K3_SET = PIN(3);
constexpr uint8_t PINC_K2_RESET = PIN(4);
constexpr uint8_t PINC_K2_SET = PIN(5);

constexpr uint8_t PINC_OUTPUTS =
    PINC_K3_RESET | PINC_K3_SET |
    PINC_K2_RESET | PINC_K2_SET;

// PORT D
// Pins 0,1 = UART RX,TX
constexpr uint8_t PIND_UART_RX = PIN(0);
constexpr uint8_t PIND_UART_TX = PIN(1);

constexpr uint8_t PIND_ONEWIRE = PIN(4); // with exteral pullup

constexpr uint8_t PIND_K1_RESET = PIN(5);
constexpr uint8_t PIND_K1_SET = PIN(6);

constexpr uint8_t PIND_OUTPUTS =
    PIND_UART_TX |
    PIND_K1_RESET | PIND_K1_SET;

/* Storage of battery capacity in mC.
 * 3600 mC = 1mAh */

/* Store the capacity three times in EEPROM, and check data validity using majority vote */
uint32_t EEMEM stored_capacity1;
uint32_t EEMEM stored_capacity2;
uint32_t EEMEM stored_capacity3;
uint32_t last_store_time; /* In seconds */

uint32_t current_capacity;

/* Timer at approximately 100ms */
volatile uint8_t timer_counter; /* Timer in 100ms steps */
volatile uint32_t timer_seconds; /* Timer in seconds */

/* At reset, save the mcusr register to find out why we got reset.
 * Datasheet 11.9.1, example code from wdt.h */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

ISR(TIMER0_COMPA_vect)
{
    timer_counter++;

    if (timer_counter >= 10) {
        timer_seconds++;
        timer_counter = 0;
    }
}

enum class error_type_t {
    EEPROM_READ_WARNING,
    EEPROM_READ_ERROR,
    EEPROM_WRITE_ERROR,
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
    snprintf(timestamp_buf, 15, "TEXT,%ld,", timer_seconds);
    uart_puts(timestamp_buf);
    uart_puts(message);
    uart_puts_P(ENDL);
}

static void flag_error(const error_type_t e)
{
    snprintf(timestamp_buf, 15, "ERROR,%ld,", timer_seconds);
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
    PORTB = PINB_STATUSn | PINB_SPI_LTC_CSn;
    PORTC = 0;
    PORTD = 0;

    // Enable output
    DDRB = PINB_OUTPUTS;
    DDRC = PINC_OUTPUTS;
    DDRD = PIND_OUTPUTS;

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
    timer_seconds = 0;
    timer_counter = 0;
    TCCR0B |= (1 << WGM02); // Set timer mode to CTC (datasheet 15.7.2)
    TIMSK0 |= (1 << TOIE0); // enable overflow interrupt
    OCR0A = (uint8_t)(F_CPU / 1024 / 10); // Overflow at 99.84 ms
    TCCR0B |= (1 << CS02) | (1 << CS00); // Start timer at Fcpu/1024

    /* Load capacity stored in EEPROM */
    load_capacity_from_eeprom();
    last_store_time = timer_seconds;

    /* Enable interrupts */
    sei();

    /* Put the CPU to sleep */
    set_sleep_mode(SLEEP_MODE_IDLE);
    while (true) {
        sleep_mode();

        if (last_store_time + 3600 * 5 >= timer_seconds) {
            store_capacity_to_eeprom();
        }
    }

    return 0;
}
