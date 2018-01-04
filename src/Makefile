CC     = gcc
CFLAGS = -std=gnu99 -Wall -Wextra -c
EXEC   = client image tacho tacho_local main main_local

.PHONY: run clean ultraclean

run:
	./main

#################### MAIN ####################
main: main.o client.o config.o image.o position.o sensors.o tacho.o
	$(CC) main.o client.o config.o image.o position.o sensors.o tacho.o -o main -lm -lbluetooth -lpthread -lev3dev-c

main.o: main.c main.h const.h config.h tacho.h sensors.h position.h image.h client.h
	$(CC) -I./ev3dev-c/source/ev3 -O2 -Wno-comment $(CFLAGS) main.c -o main.o

main_local: main.o_local client.o config.o_local image.o position.o sensors.o_local tacho.o_local
	$(CC) main.o_local client.o config.o_local image.o position.o sensors.o_local tacho.o_local -o main_local -lm -lbluetooth -lpthread

main.o_local: main.c main.h const.h config.h tacho.h sensors.h position.h image.h client.h
	$(CC) -I./../../ev3dev-c/source/ev3 $(CFLAGS) main.c -o main.o_local

#################### CLIENT ####################
client: client.o
	$(CC) client.o -o client -lm -lbluetooth

client.o: client.c client.h const.h
	$(CC) $(CFLAGS) client.c -o client.o

#################### CONFIG ####################
config.o: config.c config.h const.h client.h image.h
	$(CC) $(CFLAGS) config.c -o config.o

config.o_local: config.c config.h const.h client.h image.h
	$(CC) -I./../../ev3dev-c/source/ev3 $(CFLAGS) config.c -o config.o_local

#################### IMAGE ####################
image: image.o client.o
	$(CC) image.o client.o -o image -lm -lbluetooth

image.o: image.c image.h const.h
	$(CC) $(CFLAGS) image.c -o image.o

#################### POSITION ####################
position.o: position.c position.h const.h client.h
	$(CC) $(CFLAGS) position.c -o position.o

#################### SENSORS ####################
sensors.o: sensors.c sensors.h const.h
	$(CC) $(CFLAGS) sensors.c -o sensors.o

sensors.o_local: sensors.c sensors.h const.h
	$(CC) -I./../../ev3dev-c/source/ev3 $(CFLAGS) sensors.c -o sensors.o_local

#################### TACHO ####################
tacho: tacho.o position.o sensors.o client.o
	$(CC) tacho.o position.o sensors.o client.o -o tacho -lm -lbluetooth -lpthread -lev3dev-c

tacho_local: tacho.o_local position.o sensors.o_local
	$(CC) tacho.o_local position.o sensors.o_local -o tacho_local -lm -lpthread

tacho.o: tacho.c tacho.h const.h position.h sensors.h
	$(CC) $(CFLAGS) tacho.c -o tacho.o

tacho.o_local: tacho.c tacho.h const.h position.h sensors.h
	$(CC) -I./../../ev3dev-c/source/ev3 $(CFLAGS) tacho.c -o tacho.o_local

clean:
	rm -f *.o *.o_local

ultraclean:
	rm -f *.o *.o_local $(EXEC)
