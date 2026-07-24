CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11 -Isrc
LDLIBS  := -lpthread -lm

SRC := $(wildcard src/*.c)
OBJ := $(SRC:.c=.o)
BIN := hdhr-emu

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

# RF signal-stat calibration sweep — see tools/calibrate_stats.c.
# Linked directly against just the frontend/channel-table sources
# (not via the src/*.c wildcard above) so it doesn't pull in main.c's
# main() too. Usage: ./calibrate <dvb-adapter-num> [frontend-num]
calibrate: src/dvb_frontend.c src/dvb_frontend.h src/atsc_freq.c src/atsc_freq.h tools/calibrate_stats.c
	$(CC) $(CFLAGS) -o calibrate src/dvb_frontend.c src/atsc_freq.c tools/calibrate_stats.c $(LDLIBS)

clean:
	rm -f $(OBJ) $(BIN) calibrate

# Installs the binary + a systemd unit. Run as root on the target Pi.
install: $(BIN)
	install -Dm755 $(BIN) /usr/local/bin/$(BIN)
	install -Dm644 systemd/hdhr-emu.service /etc/systemd/system/hdhr-emu.service
	[ -f /etc/hdhr-emu.conf ] || install -Dm644 config/hdhr-emu.conf.example /etc/hdhr-emu.conf
	setcap 'cap_net_bind_service=+ep' /usr/local/bin/$(BIN) || true
	systemctl daemon-reload
	@echo "Edit /etc/hdhr-emu.conf, then: systemctl enable --now hdhr-emu"
