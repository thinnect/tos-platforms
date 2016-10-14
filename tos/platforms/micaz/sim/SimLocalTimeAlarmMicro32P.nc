/*
 * Copyright (c) 2014, Defendec.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Andres Vahter <andres.vahter@defendec.com>
 */

module SimLocalTimeAlarmMicro32P {
    provides {
        interface Alarm<TMicro, uint32_t>;
        interface LocalTime<TMicro>;
    }
}
implementation {

    sim_event_t* m_timer_event;
    bool m_running = FALSE;
    uint32_t m_alarm_fire_time = 0;

    /* m_last_zero keeps track of the phase of the clock. It denotes the sim
     * time at which the underlying clock started, which is needed to
     * calculate when compares will occur.
     */
    sim_time_t m_last_zero = 0;

    sim_time_t last_zero() {
        if (m_last_zero == 0) {
            m_last_zero = sim_mote_start_time(sim_node());
        }
        return m_last_zero;
    }

    sim_time_t notify_clock_ticks_per_sec() {
        return 1000000UL;
    }

    sim_time_t clock_to_sim(sim_time_t t) {
        t *= sim_ticks_per_sec();
        t /= notify_clock_ticks_per_sec();
        return t;
    }

    sim_time_t sim_to_clock(sim_time_t t) {
        t *= notify_clock_ticks_per_sec();
        t /= sim_ticks_per_sec();
        return t;
    }

    void timer_event_handle(sim_event_t* evt) {
        if (evt->cancelled) {
            return;
        }
        else {
            m_running = FALSE;
            signal Alarm.fired();
        }
    }

    sim_event_t* allocate_timer_event() {
        sim_event_t* new_event = sim_queue_allocate_event();

        new_event->handle = timer_event_handle;
        new_event->cleanup = sim_queue_cleanup_none;
        return new_event;
    }

    async command void Alarm.start(uint32_t dt) {
        m_timer_event = allocate_timer_event();
        m_timer_event->time = sim_time() + clock_to_sim(dt);
        m_running = TRUE;
        sim_queue_insert(m_timer_event);
    }


    async command void Alarm.stop() {
        m_timer_event->cancelled = TRUE;
        m_running = FALSE;
    }


    async command bool Alarm.isRunning() {
        return m_running;
    }


    async command void Alarm.startAt(uint32_t t0, uint32_t dt) {
        m_timer_event = allocate_timer_event();
        m_timer_event->time = sim_time() + clock_to_sim(t0) + clock_to_sim(dt);
        m_running = TRUE;
        sim_queue_insert(m_timer_event);
    }


    async command uint32_t Alarm.getNow() {
        sim_time_t elapsed = sim_time() - last_zero();
        elapsed = sim_to_clock(elapsed);
        return elapsed;
    }


    async command uint32_t Alarm.getAlarm() {
        return m_alarm_fire_time;
    }

    async command uint32_t LocalTime.get() {
        sim_time_t elapsed = sim_time() - last_zero();
        elapsed = sim_to_clock(elapsed);
        return elapsed;
    }
}
