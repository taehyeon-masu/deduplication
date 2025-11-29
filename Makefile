CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Iinclude

SRC_DIR := src
BIN     := dedup_bin

SRCS    := main.c \
           $(SRC_DIR)/bin_io.c \
           $(SRC_DIR)/compressor.c \
           $(SRC_DIR)/dictionary.c

OBJS    := $(SRCS:.c=.o)

.PHONY: all clean

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(BIN)