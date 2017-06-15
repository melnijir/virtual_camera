CC=gcc
FLAGS=-O2 -Wall -Wextra -pedantic -I../module -std=c99


all: vcctrl

vcctrl: vcctrl.c
	$(CC) $(FLAGS) vcctrl.c -o vcctrl

clean:
	@rm -rf vcctrl

.PHONY: all clean