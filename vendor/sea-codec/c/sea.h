/*
    SEA - Simple Embedded Audio Codec
    Copyright (C) 2025 Dani Biró
    MIT License

    WARNING: This is just a proof of concept code, without error checking. Use it at your own risk.
    The Rust version is much more robust and has error checking.
*/

#ifndef SEA_H
#define SEA_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEA_MIN(a, b) ((a) < (b) ? (a) : (b))
#define SEAC_MAGIC_REV 0x63616573 // 'seac' in little endian

#define SEA_DIV_CEIL(a, b) ((a) + (b) - 1) / (b)
#define SEA_CLAMP_I16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))

#define SEA_READ_U8(b) (*(*b)++)
#define SEA_READ_I16_LE(b) (*b += 2, (int16_t)((*b)[-2] | ((*b)[-1] << 8)))
#define SEA_READ_U16_LE(b) (*b += 2, (uint16_t)((*b)[-2] | ((*b)[-1] << 8)))
#define SEA_READ_U32_LE(b) (*b += 4, (uint32_t)((*b)[-4] | ((*b)[-3] << 8) | ((*b)[-2] << 16) | ((*b)[-1] << 24)))

typedef struct {
    int32_t history[4];
    int32_t weights[4];
} SEA_LMS;

static void sea_read_unpack_bits(uint8_t bit_size, const uint8_t** encoded, uint32_t bytes_to_read, uint8_t* output)
{
    const uint32_t MASKS[9] = { 0, 1, 3, 7, 15, 31, 63, 127, 255 };
    uint32_t bits_stored = 0, carry = 0;
    uint32_t output_len = 0;

    for (int i = 0; i < bytes_to_read; i++) {
        uint32_t v = (carry << 8) | SEA_READ_U8(encoded);
        bits_stored += 8;
        while (bits_stored >= bit_size) {
            output[output_len++] = (v >> (bits_stored - bit_size)) & MASKS[bit_size];
            bits_stored -= bit_size;
        }
        carry = v & ((1 << bits_stored) - 1);
    }
}

static int32_t* SEA_DQT = NULL;
static uint32_t SEA_DQT_COLUMNS = 0;
static uint32_t SEA_DQT_SCALE_FACTOR_BITS = 0;
static uint32_t SEA_DQT_RESIDUAL_BITS = 0;

static void sea_alloc_prepare_dqt(uint32_t scale_factor_bits, uint32_t residual_bits)
{
    if (SEA_DQT_SCALE_FACTOR_BITS == scale_factor_bits && SEA_DQT_RESIDUAL_BITS == residual_bits) {
        return;
    }

    if (SEA_DQT != NULL) {
        free(SEA_DQT);
    }

    static const float IDEAL_POW_FACTOR[8] = { 12.0f, 11.65f, 11.20f, 10.58f, 9.64f, 8.75f, 7.66f, 6.63f };

    uint32_t scale_factor_items = 1 << scale_factor_bits;
    uint32_t dqt_len = 1 << (residual_bits - 1);

    float power_factor = IDEAL_POW_FACTOR[residual_bits - 1] / (float)scale_factor_bits;
    int32_t scale_factors[256];
    for (uint32_t i = 0; i < scale_factor_items; ++i) {
        scale_factors[i] = (int32_t)powf((float)(i + 1), power_factor);
    }

    float dqt[128];
    if (residual_bits == 1) {
        dqt[0] = 2.0f;
    } else if (residual_bits == 2) {
        dqt[0] = 1.115f;
        dqt[1] = 4.0f;
    } else {
        dqt[0] = 0.75f;
        float end = (float)((1 << residual_bits) - 1);
        float step = floorf((end - dqt[0]) / (dqt_len - 1));
        for (uint32_t i = 1; i < dqt_len - 1; ++i) {
            dqt[i] = 0.5f + i * step;
        }
        dqt[dqt_len - 1] = end;
    }

    SEA_DQT = (int32_t*)malloc(scale_factor_items * dqt_len * 2 * sizeof(int32_t));
    uint32_t idx = 0;
    for (uint32_t s = 0; s < scale_factor_items; ++s) {
        for (uint32_t q = 0; q < dqt_len; ++q) {
            int32_t val = (int32_t)roundf(scale_factors[s] * dqt[q]);
            SEA_DQT[idx++] = val;
            SEA_DQT[idx++] = -val;
        }
    }

    SEA_DQT_COLUMNS = dqt_len * 2;
}

static inline int32_t sea_lms_predict(const SEA_LMS* lms)
{
    int32_t prediction = 0;
    for (int i = 0; i < 4; ++i) {
        prediction += lms->weights[i] * lms->history[i];
    }
    return prediction >> 13;
}

static inline void sea_lms_update(SEA_LMS* lms, int16_t sample, int32_t residual)
{
    int32_t delta = residual >> 4;
    for (int i = 0; i < 4; ++i) {
        lms->weights[i] += lms->history[i] < 0 ? -delta : delta;
    }
    for (int i = 1; i < 4; ++i) {
        lms->history[i - 1] = lms->history[i];
    }
    lms->history[3] = (int32_t)sample;
}

static int sea_read_chunk(const uint8_t** encoded, uint32_t channels, uint32_t frames_in_this_chunk, int16_t** output)
{
    uint8_t type = SEA_READ_U8(encoded);
    if (type != 0x01) {
        fprintf(stderr, "Only CBR supported\n");
        return 1;
    }
    uint8_t scale_factor_and_residual_size = SEA_READ_U8(encoded);
    uint8_t scale_factor_bits = scale_factor_and_residual_size >> 4;
    uint8_t residual_size = scale_factor_and_residual_size & 0xF;
    uint8_t scale_factor_frames = SEA_READ_U8(encoded);
    uint8_t reserved = SEA_READ_U8(encoded);
    if (reserved != 0x5A) {
        fprintf(stderr, "Invalid file\n");
        return 1;
    }

    sea_alloc_prepare_dqt(scale_factor_bits, residual_size);

    SEA_LMS* lms = (SEA_LMS*)malloc(channels * sizeof(SEA_LMS));
    for (int channel_id = 0; channel_id < channels; channel_id++) {
        for (int j = 0; j < 4; j++) {
            lms[channel_id].history[j] = SEA_READ_I16_LE(encoded);
        }
        for (int j = 0; j < 4; j++) {
            lms[channel_id].weights[j] = SEA_READ_I16_LE(encoded);
        }
    }

    uint32_t scale_factor_items = SEA_DIV_CEIL(frames_in_this_chunk, scale_factor_frames) * channels;
    uint8_t* scale_factors = (uint8_t*)malloc(scale_factor_items + 8);
    uint32_t scale_factor_bytes = SEA_DIV_CEIL(scale_factor_items * scale_factor_bits, 8);
    sea_read_unpack_bits(scale_factor_bits, encoded, scale_factor_bytes, scale_factors);

    uint32_t residual_bytes = SEA_DIV_CEIL(frames_in_this_chunk * residual_size * channels, 8);
    uint8_t* residuals = (uint8_t*)malloc(frames_in_this_chunk * channels + 8);
    sea_read_unpack_bits(residual_size, encoded, residual_bytes, residuals);

    for (int scale_factor_offset = 0; scale_factor_offset < scale_factor_items; scale_factor_offset += channels) {
        uint8_t* scale_factor_residuals = &residuals[scale_factor_offset * scale_factor_frames];
        for (int frame_index = 0; frame_index < scale_factor_frames; frame_index++) {
            const uint8_t* subchunk_residuals = &scale_factor_residuals[frame_index * channels];
            for (int channel_index = 0; channel_index < channels; ++channel_index) {
                uint8_t scale_factor = scale_factors[scale_factor_offset + channel_index];
                int32_t predicted = sea_lms_predict(&lms[channel_index]);
                uint32_t quantized = (uint32_t)subchunk_residuals[channel_index];
                int32_t dequantized = SEA_DQT[scale_factor * SEA_DQT_COLUMNS + quantized];
                int32_t reconstructed = SEA_CLAMP_I16(predicted + dequantized);
                **output = reconstructed;
                *output += 1;
                sea_lms_update(&lms[channel_index], reconstructed, dequantized);
            }
        }
    }

    free(residuals);
    free(scale_factors);
    free(lms);
    return 0;
}

int sea_decode(uint8_t* encoded, uint32_t encoded_len, uint32_t* sample_rate, uint32_t* channels, int16_t* output, uint32_t* total_frames)
{
    const uint8_t** encoded_ptr = (const uint8_t**)&encoded;

    uint32_t magic = SEA_READ_U32_LE(encoded_ptr);
    uint8_t version = SEA_READ_U8(encoded_ptr);

    if (magic != SEAC_MAGIC_REV || version != 1) {
        fprintf(stderr, "Invalid file\n");
        return 1;
    }

    *channels = SEA_READ_U8(encoded_ptr);
    uint16_t chunk_size = SEA_READ_U16_LE(encoded_ptr);
    uint16_t frames_per_chunk = SEA_READ_U16_LE(encoded_ptr);
    *sample_rate = SEA_READ_U32_LE(encoded_ptr);
    *total_frames = SEA_READ_U32_LE(encoded_ptr);
    uint32_t metadata_len = SEA_READ_U32_LE(encoded_ptr);
    encoded_ptr += metadata_len;

    if (output == NULL) {
        return 0;
    }

    uint32_t read_frames = 0;
    int16_t** output_ptr = (int16_t**)&output;
    while (read_frames < *total_frames) {
        uint32_t frames_in_chunk = SEA_MIN(frames_per_chunk, *total_frames - read_frames);
        uint32_t written_samples = sea_read_chunk(encoded_ptr, *channels, frames_in_chunk, output_ptr);
        if (written_samples != 0) {
            fprintf(stderr, "Decode error\n");
            return 2;
        }
        read_frames += frames_in_chunk;
    }

    free(SEA_DQT);
    return 0;
}

#endif
