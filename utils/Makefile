CC=gcc
FLAGS=-O2 -Wall -Wextra -pedantic -I../module -std=c99
PREFIX=/usr

.PHONY: all
all: vcctrl

vcctrl: vcctrl.c
	$(CC) $(FLAGS) vcctrl.c -o vcctrl

.PHONY: clean
clean:
	@rm -rf vcctrl

.PHONY: install
install:
	mkdir -p $(DESTDIR)$(PREFIX)/bin/
	mkdir -p $(DESTDIR)$(PREFIX)/include/
	install -m 755 vcctrl $(DESTDIR)$(PREFIX)/bin/
	install -m 644 ../module/vcmod_api.h $(DESTDIR)$(PREFIX)/include