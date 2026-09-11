CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -O2
OBJS    = Main.o API.o maze.o

all: mouse

mouse: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o mouse

Main.o: Main.c API.h maze.h
API.o:  API.c API.h

maze.o: maze.c API.h maze.h

clean:
	rm -f mouse mouse.exe $(OBJS)

.PHONY: all clean
