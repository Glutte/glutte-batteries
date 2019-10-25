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

#pragma once

#include <stdlib.h>
#include <stdint.h>

// Definitions allocation pin
#define PIN(x) (1 << x)

/* All relay signals PINx_Kx have external pulldown.
 * All pins whose name ends in 'n' are active low */

// For arduino pin numbers (not used in this file), see
// lib/pins_arduino.h

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
