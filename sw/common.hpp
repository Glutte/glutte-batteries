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
    uint32_t seconds_ = 0;
    uint32_t microsecs_ = 0;

    timer_t() {}
    timer_t(uint32_t seconds, uint32_t microsecs) : seconds_(seconds), microsecs_(microsecs) {}

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

    uint32_t get_microsecs_atomic() const {
        cli();
        uint32_t t = microsecs_;
        sei();
        return t;
    }

    bool operator>(const timer_t& rhs) const {
        return (seconds_ > rhs.seconds_) or
            (seconds_ == rhs.seconds_ and microsecs_ > rhs.microsecs_);
    }

    bool operator<(const timer_t& rhs) const {
        return (seconds_ < rhs.seconds_) or
            (seconds_ == rhs.seconds_ and microsecs_ < rhs.microsecs_);
    }

    void normalise() {
        while (microsecs_ >= 1000000uL) {
            seconds_++;
            microsecs_ -= 1000000uL;
        }
    }

    timer_t operator+(const timer_t& rhs) const {
        timer_t t;
        t.seconds_ = seconds_ + rhs.seconds_;
        t.microsecs_ = microsecs_ + rhs.microsecs_;
        t.normalise();
        return t;
    }

    void operator+=(const timer_t& inc) {
        seconds_ += inc.seconds_;
        microsecs_ += inc.microsecs_;
        normalise();
    }

    void operator+=(uint32_t mirosecs) {
        *this += timer_t(0, mirosecs);
    }
};


enum class relay_id_t {
    K1,
    K2,
    K3,
};
