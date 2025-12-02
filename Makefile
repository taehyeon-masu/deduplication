CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Iinclude

SRC_DIR := src

BIN_DEDUP := dedup_bin
BIN_PACK  := pack_trhp

SRCS_DEDUP := main.c \
              $(SRC_DIR)/bin_io.c \
              $(SRC_DIR)/compressor.c \
              $(SRC_DIR)/dictionary.c

SRCS_PACK  := $(SRC_DIR)/pack_trhp.c

OBJS_DEDUP := $(SRCS_DEDUP:.c=.o)
OBJS_PACK  := $(SRCS_PACK:.c=.o)

.PHONY: all clean

all: $(BIN_DEDUP) $(BIN_PACK)

$(BIN_DEDUP): $(OBJS_DEDUP)
	$(CC) $(CFLAGS) -o $@ $^

$(BIN_PACK): $(OBJS_PACK)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS_DEDUP) $(OBJS_PACK) $(BIN_DEDUP) $(BIN_PACK)
