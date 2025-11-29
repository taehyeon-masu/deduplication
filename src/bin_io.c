#include "../include/bin_io.h"
#include <stdio.h>
#include <stdlib.h>

int read_binary_file(const char *filename,
                     unsigned char **data_out,
                     size_t *nbytes_out)
{
    FILE *fp = fopen(filename, "rb");
    if (!fp)
    {
        perror("fopen input");
        return 1;
    }

    if (fseek(fp, 0, SEEK_END) != 0)
    {
        perror("fseek");
        fclose(fp);
        return 1;
    }
    long sz = ftell(fp);
    if (sz < 0)
    {
        perror("ftell");
        fclose(fp);
        return 1;
    }
    rewind(fp);

    unsigned char *buf = (unsigned char *)malloc((size_t)sz);
    if (!buf)
    {
        fprintf(stderr, "Failed to allocate buffer for file\n");
        fclose(fp);
        return 1;
    }

    size_t nread = fread(buf, 1, (size_t)sz, fp);
    if (nread != (size_t)sz)
    {
        fprintf(stderr, "Failed to read entire file\n");
        free(buf);
        fclose(fp);
        return 1;
    }
    fclose(fp);

    *data_out = buf;
    *nbytes_out = nread;
    return 0;
}

int write_binary_file(const char *filename,
                      const unsigned char *data,
                      size_t nbytes)
{
    FILE *fp = fopen(filename, "wb");
    if (!fp)
    {
        perror("fopen output");
        return 1;
    }
    size_t nwritten = fwrite(data, 1, nbytes, fp);
    if (nwritten != nbytes)
    {
        fprintf(stderr, "Failed to write all bytes\n");
        fclose(fp);
        return 1;
    }
    fclose(fp);
    return 0;
}
