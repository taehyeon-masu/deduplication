#ifndef COMPRESSOR_H
#define COMPRESSOR_H

int compress_file(const char *input_filename,
                  const char *output_filename,
                  int width_bytes,
                  int block_size_samples);

int compress_file_multi(const char *input_filename,
                        const char *output_filename,
                        int num_segs,
                        const int *seg_sizes);

int decompress_file(const char *input_filename,
                    const char *output_filename);

#endif