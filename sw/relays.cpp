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

#include "relays.hpp"
#include "pins.hpp"
#include <stdio.h>
#include <math.h>

struct pending_event_t {
    timer_t when;
    relay_id_t relay;
    bool set_not_reset;
    bool level;
    bool pending;
};

static constexpr int RELAY_SIGNAL_HOLD_TIME_MS = 400;

static constexpr size_t PENDING_EVENTS_SIZE = 8;

static pending_event_t pending_events[PENDING_EVENTS_SIZE];

void relays_init()
{
    for (size_t i = 0; i < PENDING_EVENTS_SIZE; i++) {
        pending_events[i].pending = false;
    }
}

static void handle_event(pending_event_t& event)
{
    switch (event.relay) {
        case relay_id_t::K1:
            if (event.level) {
                PORTD |= (event.set_not_reset ? PIND_K1_SET : PIND_K1_RESET);
            }
            else {
                PORTD &= (event.set_not_reset ? ~PIND_K1_SET : ~PIND_K1_RESET);
            }
            break;
        case relay_id_t::K2:
            if (event.level) {
                PORTC |= (event.set_not_reset ? PINC_K2_SET : PINC_K2_RESET);
            }
            else {
                PORTC &= (event.set_not_reset ? ~PINC_K2_SET : ~PINC_K2_RESET);
            }
            break;
        case relay_id_t::K3:
            if (event.level) {
                PORTC |= (event.set_not_reset ? PINC_K3_SET : PINC_K3_RESET);
            }
            else {
                PORTC &= (event.set_not_reset ? ~PINC_K3_SET : ~PINC_K3_RESET);
            }
            break;
    }

    event.pending = false;
}

void relays_handle(const timer_t& time_now)
{
    for (size_t i = 0; i < PENDING_EVENTS_SIZE; i++) {
        if (pending_events[i].pending and pending_events[i].when < time_now) {
            handle_event(pending_events[i]);
        }
    }
}

bool relays_toggle(relay_id_t relay, bool set_not_reset, const timer_t& when)
{
    size_t num_free_events = 0;

    for (size_t i = 0; i < PENDING_EVENTS_SIZE; i++) {
        if (not pending_events[i].pending) {
            num_free_events++;
        }
    }

    if (num_free_events < 2) {
        return false;
    }

    for (size_t i = 0; i < PENDING_EVENTS_SIZE; i++) {
        if (not pending_events[i].pending) {
            pending_events[i].pending = true;
            pending_events[i].relay = relay;
            pending_events[i].when = when;
            pending_events[i].level = true;
            pending_events[i].set_not_reset = set_not_reset;
            break;
        }
    }

    for (size_t i = 0; i < PENDING_EVENTS_SIZE; i++) {
        if (not pending_events[i].pending) {
            pending_events[i].pending = true;
            pending_events[i].relay = relay;
            pending_events[i].when = when +
              timer_t{0, timer_t::ms_to_ticks(RELAY_SIGNAL_HOLD_TIME_MS)};

            pending_events[i].level = false;
            pending_events[i].set_not_reset = set_not_reset;
            break;
        }
    }
    return true;
}

