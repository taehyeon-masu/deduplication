#ifndef COMPRESSOR_H
#define COMPRESSOR_H

#include <stddef.h>

/* dev_top_n: 두 번째 segment부터 deviation으로 쓸 top-N 비트 개수 */
int compress_file(const char *input_filename,
                  const char *output_filename,
                  int num_fields,
                  const int *field_sizes,
                  int dev_top_n);

int decompress_file(const char *input_filename,
                    const char *output_filename);

#endif /* COMPRESSOR_H */
