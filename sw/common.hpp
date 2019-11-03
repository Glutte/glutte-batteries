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
#include <avr/interrupt.h>

struct timer_t {
    uint32_t seconds_ = 0; /* Timer in seconds */
    uint8_t ticks_ = 0; /* Timer in 100ms steps */

    timer_t() {}
    timer_t(uint32_t seconds, uint8_t ticks) : seconds_(seconds), ticks_(ticks) {}

    timer_t get_atomic_copy() const {
        cli();
        const auto t = *this;
        sei();
        return t;
    }

    uint32_t get_seconds_atomic() const {
        cli();
        uint32_t s = seconds_;
        sei();
        return s;
    }

    uint8_t get_ticks_atomic() const {
        /* Returning an uint8_t is atomic */
        return ticks_;
    }

    bool operator>(const timer_t& rhs) const {
        return (seconds_ > rhs.seconds_) or
            (seconds_ == rhs.seconds_ and ticks_ > rhs.ticks_);
    }

    bool operator<(const timer_t& rhs) const {
        return (seconds_ < rhs.seconds_) or
            (seconds_ == rhs.seconds_ and ticks_ < rhs.ticks_);
    }

    void normalise() {
        while (ticks_ >= 10) {
            seconds_++;
            ticks_ -= 10;
        }
    }

    timer_t operator+(const timer_t& rhs) const {
        timer_t t;
        t.seconds_ = seconds_ + rhs.seconds_;
        t.ticks_ = ticks_ + rhs.ticks_;
        t.normalise();
        return t;
    }

    timer_t operator+(uint8_t ticks) const {
        timer_t t = timer_t(0, ticks);
        return *this + t;
    }

    void operator+=(const timer_t& inc) {
        seconds_ += inc.seconds_;
        ticks_ += inc.ticks_;
        normalise();
    }

    void operator+=(uint8_t ticks) {
        *this += timer_t(0, ticks);
    }

    static constexpr int ms_to_ticks(int ms) { return ms / 100; }
};


enum class relay_id_t {
    K1,
    K2,
    K3,
};
