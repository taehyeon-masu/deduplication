#ifndef BIN_IO_H
#define BIN_IO_H

#include <stddef.h>

int read_binary_file(const char *filename, unsigned char **data_out, size_t *nbytes_out);

int write_binary_file(const char *filename, const unsigned char *data, size_t nbytes);

#endif