#include "sea.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define WAV_HEADER_SIZE 44

void write_wav_header(FILE* file, uint32_t sample_rate, uint32_t channels, uint32_t num_frames)
{
    uint32_t byte_rate = sample_rate * channels * 2;
    uint32_t data_size = num_frames * channels * 2;
    uint32_t chunk_size = 36 + data_size;

    fwrite("RIFF", 1, 4, file);
    fwrite(&chunk_size, 4, 1, file);
    fwrite("WAVE", 1, 4, file);
    fwrite("fmt ", 1, 4, file);

    uint32_t subchunk1_size = 16;
    uint16_t audio_format = 1;
    uint16_t num_channels = (uint16_t)channels;
    uint32_t sample_rate_le = sample_rate;
    uint16_t bits_per_sample = 16;
    uint16_t block_align = num_channels * 2;
    uint32_t byte_rate_le = byte_rate;

    fwrite(&subchunk1_size, 4, 1, file);
    fwrite(&audio_format, 2, 1, file);
    fwrite(&num_channels, 2, 1, file);
    fwrite(&sample_rate_le, 4, 1, file);
    fwrite(&byte_rate_le, 4, 1, file);
    fwrite(&block_align, 2, 1, file);
    fwrite(&bits_per_sample, 2, 1, file);

    fwrite("data", 1, 4, file);
    fwrite(&data_size, 4, 1, file);
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input_file> <output_file>\n", argv[0]);
        return 1;
    }

    FILE* input_file = fopen(argv[1], "rb");
    if (!input_file) {
        perror("Error opening input file");
        return 1;
    }

    fseek(input_file, 0, SEEK_END);
    uint32_t encoded_len = ftell(input_file);
    rewind(input_file);

    uint8_t* encoded = (uint8_t*)malloc(encoded_len);
    fread(encoded, 1, encoded_len, input_file);
    fclose(input_file);

    uint32_t sample_rate, channels, output_frames;
    sea_decode(encoded, encoded_len, &sample_rate, &channels, NULL, &output_frames);

    int16_t* output = (int16_t*)malloc(output_frames * channels * sizeof(int16_t));
    sea_decode(encoded, encoded_len, &sample_rate, &channels, output, &output_frames);
    free(encoded);

    FILE* output_file = fopen(argv[2], "wb");
    if (!output_file) {
        perror("Error opening output file");
        return 1;
    }

    write_wav_header(output_file, sample_rate, channels, output_frames);
    fwrite(output, sizeof(int16_t), output_frames * channels, output_file);
    fclose(output_file);

    free(output);
    printf("Decoding complete. Output written to %s\n", argv[2]);
    return 0;
}
