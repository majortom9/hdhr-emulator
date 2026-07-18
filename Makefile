CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11 -Isrc
LDLIBS  := -lpthread -lm

SRC := $(wildcard src/*.c)
OBJ := $(SRC:.c=.o)
BIN := hdhr-emulator

.PHONY: all clean install test

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# PAT/PMT/TVCT parser round-trip self-test — no hardware needed, pure
# data-structure encode/decode. See test/psip_test.c for what this does
# and doesn't prove.
test:
	$(CC) $(CFLAGS) -o /tmp/hdhr_psip_test src/psip.c test/psip_test.c
	/tmp/hdhr_psip_test

clean:
	rm -f $(OBJ) $(BIN)

# Installs the binary + a systemd unit. Run as root on the target Pi.
install: $(BIN)
	install -Dm755 $(BIN) /usr/local/bin/$(BIN)
	install -Dm644 systemd/hdhr-emulator.service /etc/systemd/system/hdhr-emulator.service
	[ -f /etc/hdhr-emulator.conf ] || install -Dm644 config/hdhr-emulator.conf.example /etc/hdhr-emulator.conf
	setcap 'cap_net_bind_service=+ep' /usr/local/bin/$(BIN) || true
	systemctl daemon-reload
	@echo "Edit /etc/hdhr-emulator.conf, then: systemctl enable --now hdhr-emulator"
