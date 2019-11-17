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

/* Essential information from datasheet:
 *
 * SCK mode:
 * The internal or external SCK mode is selected on power-up and then
 * reselected every time a HIGH-to-LOW transition is detected at the CS pin. If
 * SCK is HIGH or floating at power-up or during this transition, the
 * converter enters the internal SCK mode. If SCK is LOW at power-up or
 * during this transition, the converter enters the external SCK mode.
 *
 * CS:
 * At any time during the conversion cycle, CS may be pulled LOW in order to
 * monitor the state of the converter.  While CS is pulled LOW, EOC is output
 * to the SDO pin. EOC = 1 while a conversion is in progress and EOC = 0 if the
 * device is in the sleep state. Independent of CS, the device automatically
 * enters the low power sleep state once the conversion is complete.
 *
 * The device remains in the sleep state until the first rising edge of SCK is
 * seen while CS is LOW.
 *
 * We must latch incoming data on the rising edge of SCK.
 *
 * On the 32nd falling edge of SCK, the device begins a new conversion.
 *
 * The sub LSBs are valid conversion results beyond the 24-bit level that may
 * be included in averaging or discarded without loss of resolution.
 *
 * Output format, 32 bits in total:
 * EOC, DMY, SIG, EXR, MSB..LSB (24 bits), SUB-LSB(4 bits)
 *
 * - EOC goes low when the conversion is complete
 * - DMY is always low
 * - SIG is 1 when Vin > 0
 * - EXR is 0 when 0 <= Vin <= Vref
 *
 *
 * We will use External SCK, Single Cycle Conversion (see table 4):
 * - SCK must be low on falling edge of CSn
 * - We will discard the sub LSB
 * - We will check that DMY is always 0, SIG always 1 and EXR always 0
 *
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ltc2400.h"

static void cs_low()
{
    cli();
    PORTB &= ~PINB_SPI_LTC_CSn;
    sei();
}

static void cs_high()
{
    cli();
    PORTB |= PINB_SPI_LTC_CSn;
    sei();
}

void ltc2400_init()
{
    cli();

    // Set SPI Enable and Master mode,
    // bit order=MSB first (DORD=0),
    // SPI mode=0 (CPOL=0, CPHA=0)
    //
    // clock divider:
    // SPR1 and SPR0 are the two LSB bits in SPCR
    // SPR1 SPR0 ~SPI2X Freq
    //   0    0     0   fosc/2
    //   0    1     0   fosc/8
    //   1    0     0   fosc/32
    //   1    1     0   fosc/64
    // double speed mode:
    //   0    0     1   fosc/4
    //   0    1     1   fosc/16
    //   1    0     1   fosc/64
    //   1    1     1   fosc/128
    //
    // Set SPR1 for /32
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1);
    SPSR = 0; // clear SPI2X

    sei();
}

int ltc2400_conversion_ready()
{
    cs_low();
    // EOC == 0 means conversion is complete and device in sleep mode
    const int eoc = (PORTB & PINB_SPI_MISO) ? 0 : 1;
    cs_high();

    return eoc;
}

float ltc2400_get_conversion_result(bool& dmy_fault, bool& exr_fault, uint32_t& adc_value)
{
    cs_low();

    uint8_t data[4] = {};

    for (int i = 0; i < 4; i++) {
        SPDR = 0x0; // always output 0

        while (!(SPSR & _BV(SPIF))) { /* wait */ }
        data[i] = SPDR;
    }

    cs_high();

    const uint32_t result =
        ((uint32_t)data[3] << 24) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[0]);

    dmy_fault = result & _BV(30);
    exr_fault = not (result & _BV(28));

    // Mask 4 MSB status bits, and shift out 4 sub-LSB bits
    adc_value = (result >> 4) & 0x00FFFFFF;

    // Convert ADC value to voltage
    return ((float)adc_value) / ((float)0x00FFFFFF) / 5.0f;
}

