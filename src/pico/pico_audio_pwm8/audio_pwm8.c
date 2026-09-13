/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/audio_pwm8.h"
#include "pico/audio_pwm8/sample_encoding.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

CU_REGISTER_DEBUG_PINS(audio_timing)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(audio_timing)

#define dma_intsx __CONCAT(dma_hw->ints, PICO_AUDIO_PWM8_DMA_IRQ)
#define dma_channel_set_irqx_enabled __CONCAT(__CONCAT(dma_channel_set_irq, PICO_AUDIO_PWM8_DMA_IRQ),_enabled)
#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_AUDIO_PWM8_DMA_IRQ)

struct {
    audio_buffer_t *playing_buffer;
    uint32_t freq;
#if !PICO_AUDIO_PWM8_SUPPORT_MULTI_SLICE
    uint8_t dma_channel;
    uint8_t slice_num;
    // todo support reverse L/R ?
#else
#error unsupported
#endif
} shared_state;

audio_format_t pio_pwm8_consumer_format;
audio_buffer_format_t pio_pwm8_consumer_buffer_format = {
        .format = &pio_pwm8_consumer_format,
};

static audio_buffer_t silence_buffer;

static void __isr __time_critical_func(audio_pwm8_dma_irq_handler)();

const audio_format_t *audio_pwm8_setup(const audio_format_t *intended_audio_format,
                                               const audio_pwm8_config_t *config) {
    for(uint i=0;i<config->num_channels;i++) {
        gpio_set_function(config->pins[i], GPIO_FUNC_PWM);
    }

    silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_PWM8_BUFFER_SAMPLE_LENGTH * 4);
    silence_buffer.sample_count = PICO_AUDIO_PWM8_BUFFER_SAMPLE_LENGTH;
    for(uint i=0;i<silence_buffer.sample_count;i++) {
        ((uint32_t*)silence_buffer.buffer->bytes)[i] = 0x00800080;
    }
    silence_buffer.format = &pio_pwm8_consumer_buffer_format;

    __mem_fence_release();

    uint8_t dma_channel = 6; //dma_claim_unused_channel(true);
    shared_state.dma_channel = dma_channel;

    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);

    shared_state.slice_num = pwm_gpio_to_slice_num(config->pins[0]);
    channel_config_set_dreq(&dma_config,
                            PWM_DREQ_NUM(shared_state.slice_num)
    );

    // todo only true for stereo
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    // todo only true for same pin
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_wrap(&pc, (clock_get_hz(clk_sys) + 24858) / 49716 - 1); // 270000000 / 49716 ish
    pwm_init(shared_state.slice_num, &pc, true);
    dma_channel_configure(dma_channel,
                          &dma_config,
                          &pwm_hw->slice[shared_state.slice_num].cc, // dest
                          NULL, // src
                          0, // count
                          false // trigger
    );

    irq_add_shared_handler(DMA_IRQ_x, audio_pwm8_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_channel_set_irqx_enabled(dma_channel, 1);
    return intended_audio_format;
}

static audio_buffer_pool_t *audio_pwm8_consumer;

static void update_pwm_frequency(uint32_t sample_freq) {
    uint64_t system_clock_frequency = clock_get_hz(clk_sys);
#if 0
    uint32_t best_error = UINT32_MAX;
    uint32_t best_val;
    for (uint32_t a = 1; a < 65536; a++) {
        uint32_t b = (a * system_clock_frequency) / sample_freq;
        if (b >> 16) break;
        uint32_t actual = (system_clock_frequency * a) / b;
        uint32_t rem = (system_clock_frequency * a) % b;
        printf("%d %d %d %d\n", a, b, actual, rem);
        b++;
        actual = (system_clock_frequency * a) / b;
        rem = (system_clock_frequency * a) % b;
        printf("%d %d %d %d\n", a, b, actual, rem);
    }
#endif
//    printf("setting pio freq %d\n", (int) sample_freq);
//    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
//    assert(system_clock_frequency < 0x40000000);
//    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
//    printf("System clock at %u, pwm8 clock divider 0x%x/256\n", (uint) system_clock_frequency, (uint)divider);
//    assert(divider < 0x1000000);
//    pio_sm_set_clkdiv_int_frac(audio_pio, shared_state.pio_sm, divider >> 8u, divider & 0xffu);
    // todo just for now
    if(sample_freq != 49716)panic("Unsupported frequency %d",sample_freq);
    /* DMA is paced by this PWM slice, not a DMA timer. */
    pwm_set_wrap(shared_state.slice_num,(system_clock_frequency + sample_freq/2)/sample_freq - 1);
    shared_state.freq = sample_freq;
}

static audio_buffer_t *wrap_consumer_take(audio_connection_t *connection, bool block) {
    // support dynamic frequency shifting
    if (connection->producer_pool->format->sample_freq != shared_state.freq) {
        update_pwm_frequency(connection->producer_pool->format->sample_freq);
    }
#if PICO_AUDIO_PWM8_MONO_INPUT
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    // todo for now as we don't have mono
    return audio_pwm8_mono_to_stereo_consumer_take_s16(connection, block);
#else
    return audio_pwm8_mono_to_stereo_consumer_take_s16(connection, block);
#endif
#else
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    panic_unsupported();
#else
    return audio_pwm8_stereo_to_stereo_consumer_take_s16(connection, block);
#endif
#endif
}

static void wrap_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    // support dynamic frequency shifting
    if (connection->producer_pool->format->sample_freq != shared_state.freq) {
        update_pwm_frequency(connection->producer_pool->format->sample_freq);
    }
#if PICO_AUDIO_PWM8_MONO_INPUT
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    // todo for now as we don't have mono
    return audio_pwm8_mono_to_stereo_consumer_take_s16(connection, block);
    assert(false);
//    return mono_to_mono_producer_give(connection, block);
#else
hard_assert(false);
    //return mono_to_stereo_producer_give(connection, buffer);
#endif
#else
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    panic_unsupported();
#else
    return audio_pwm8_stereo_to_stereo_blocking_give_s16(connection, buffer);
#endif
#endif
}

static struct buffer_copying_on_consumer_take_connection m2s_audio_pwm8_ct_connection = {
        .core = {
                .consumer_pool_take = wrap_consumer_take,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
                .producer_pool_give = producer_pool_give_buffer_default,
        }
};

static struct producer_pool_blocking_give_connection m2s_audio_pwm8_pg_connection = {
        .core = {
                .consumer_pool_take = consumer_pool_take_buffer_default,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
                .producer_pool_give = wrap_producer_give,
        }
};

bool audio_pwm8_connect_thru(audio_buffer_pool_t *producer, audio_connection_t *connection) {
    return audio_pwm8_connect_extra(producer, false, 2, 256, connection);
}

bool audio_pwm8_connect(audio_buffer_pool_t *producer) {
    return audio_pwm8_connect_thru(producer, NULL);
}

bool audio_pwm8_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                                 uint samples_per_buffer, audio_connection_t *connection) {
    printf("Connecting PIO pwm8 audio\n");
    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S16);
    pio_pwm8_consumer_format.format = AUDIO_BUFFER_FORMAT_PCM_S16;
    // todo we could do mono
    // todo we can't match exact, so we should return what we can do
    pio_pwm8_consumer_format.sample_freq = producer->format->sample_freq;
#if PICO_AUDIO_PWM8_MONO_OUTPUT
//    pio_pwm8_consumer_format.channel_count = 1;
//    pio_pwm8_consumer_buffer_format.sample_stride = 2;
// todo no proper mono output yet
    pio_pwm8_consumer_format.channel_count = 2;
    pio_pwm8_consumer_buffer_format.sample_stride = 4;
#else
    pio_pwm8_consumer_format.channel_count = 2;
    pio_pwm8_consumer_buffer_format.sample_stride = 4;
#endif

    audio_pwm8_consumer = audio_new_consumer_pool(&pio_pwm8_consumer_buffer_format, buffer_count, samples_per_buffer);

    update_pwm_frequency(producer->format->sample_freq);

    // todo cleanup threading
    __mem_fence_release();

    if (!connection) {
        if (producer->format->channel_count == 2) {
#if PICO_AUDIO_PWM8_MONO_INPUT
            panic("need to merge channels down\n");
#else
#if PICO_AUDIO_PWM8_MONO_OUTPUT
            panic("trying to play stereo thru mono not yet supported");
#else
            printf("Copying stereo to stereo at %d Hz\n", (int) producer->format->sample_freq);
#endif
#endif
            // todo we should support pass thru option anyway
            printf("TODO... not completing stereo audio connection properly!\n");
        } else {
#if PICO_AUDIO_PWM8_MONO_OUTPUT
            printf("Copying mono to fake mono at %d Hz\n", (int) producer->format->sample_freq);
#else
            printf("Converting mono to stereo at %d Hz\n", (int) producer->format->sample_freq);
#endif
        }
        connection = buffer_on_give ? &m2s_audio_pwm8_pg_connection.core : &m2s_audio_pwm8_ct_connection.core;
    }
    audio_complete_connection(connection, producer, audio_pwm8_consumer);
    return true;
}

static struct buffer_copying_on_consumer_take_connection m2s_audio_pwm8_connection_s8 = {
        .core = {
#if PICO_AUDIO_PWM8_MONO_OUTPUT
                .consumer_pool_take = mono_s8_to_mono_consumer_take,
#else
                .consumer_pool_take = mono_s8_to_stereo_consumer_take,
#endif
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
                .producer_pool_give = producer_pool_give_buffer_default,
        }
};

bool audio_pwm8_connect_s8(audio_buffer_pool_t *producer) {
    printf("Connecting PIO pwm8 audio (S8)\n");

    // todo we need to pick a connection based on the frequency - e.g. 22050 can be more simply upsampled to 44100
    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S8);
    pio_pwm8_consumer_format.format = AUDIO_BUFFER_FORMAT_PCM_S16;
    // todo we could do mono
    // todo we can't match exact, so we should return what we can do
    pio_pwm8_consumer_format.sample_freq = producer->format->sample_freq;
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    pio_pwm8_consumer_format.channel_count = 1;
    pio_pwm8_consumer_buffer_format.sample_stride = 2;
#else
    pio_pwm8_consumer_format.channel_count = 2;
    pio_pwm8_consumer_buffer_format.sample_stride = 4;
#endif

    // we do this on take so should do it quickly...
    uint samples_per_buffer = 256;
    // todo with take we really only need 1 buffer
    audio_pwm8_consumer = audio_new_consumer_pool(&pio_pwm8_consumer_buffer_format, 2, samples_per_buffer);

    update_pwm_frequency(producer->format->sample_freq);

    // todo cleanup threading
    __mem_fence_release();

    audio_connection_t *connection;
    if (producer->format->channel_count == 2) {
#if PICO_AUDIO_PWM8_MONO_OUTPUT
        panic("trying to play stereo thru mono not yet supported");
#endif
        // todo we should support pass thru option anyway
        printf("TODO... not completing stereo audio connection properly!\n");
        connection = &m2s_audio_pwm8_connection_s8.core;
    } else {
#if PICO_AUDIO_PWM8_MONO_OUTPUT
        printf("Copying mono to fake mono at %d Hz\n", (int) producer->format->sample_freq);
#else
        printf("Converting mono to stereo at %d Hz\n", (int) producer->format->sample_freq);
#endif
        connection = &m2s_audio_pwm8_connection_s8.core;
    }
    audio_complete_connection(connection, producer, audio_pwm8_consumer);
    return true;
}

static inline void audio_start_dma_transfer() {
    assert(!shared_state.playing_buffer);
    audio_buffer_t *ab = take_audio_buffer(audio_pwm8_consumer, false);

    shared_state.playing_buffer = ab;
    if (!ab) {
        DEBUG_PINS_XOR(audio_timing, 1);
        DEBUG_PINS_XOR(audio_timing, 2);
        DEBUG_PINS_XOR(audio_timing, 1);
        //DEBUG_PINS_XOR(audio_timing, 2);
        // just play some silence
        ab = &silence_buffer;
//    } else {
//        give_audio_buffer(audio_pwm8_consumer, shared_state.playing_buffer);
//        shared_state.playing_buffer = 0;
//        return;
    }
    assert(ab->sample_count);
    // todo better naming of format->format->format!!
    assert(ab->format->format->format == AUDIO_BUFFER_FORMAT_PCM_S16);
#if PICO_AUDIO_PWM8_MONO_OUTPUT
    assert(ab->format->format->channel_count == 1);
    assert(ab->format->sample_stride == 2);
#else
    assert(ab->format->format->channel_count == 2);
    assert(ab->format->sample_stride == 4);
#endif
    dma_channel_transfer_from_buffer_now(shared_state.dma_channel, ab->buffer->bytes, ab->sample_count);
}

// irq handler for DMA
void __isr __time_critical_func(audio_pwm8_dma_irq_handler)() {
#if PICO_AUDIO_PWM8_NOOP
    assert(false);
#else
    uint dma_channel = shared_state.dma_channel;
    if (dma_intsx & (1u << dma_channel)) {
        dma_intsx = 1u << dma_channel;
        DEBUG_PINS_SET(audio_timing, 4);
        // free the buffer we just finished
        if (shared_state.playing_buffer) {
            give_audio_buffer(audio_pwm8_consumer, shared_state.playing_buffer);
#ifndef NDEBUG
            shared_state.playing_buffer = NULL;
#endif
        }
        audio_start_dma_transfer();
        DEBUG_PINS_CLR(audio_timing, 4);
    }
#endif
}

static bool audio_enabled;

void audio_pwm8_set_enabled(bool enabled) {
    if (enabled != audio_enabled) {
#ifndef NDEBUG
        if (enabled)
        {
            puts("Enabling PIO pwm8 audio\n");
            printf("(on core %d\n", get_core_num());
        }
#endif
        irq_set_enabled(DMA_IRQ_x, enabled);

        if (enabled) {
            audio_start_dma_transfer();
        }

        pwm_set_enabled(shared_state.slice_num, enabled);
        audio_enabled = enabled;
    }
}
