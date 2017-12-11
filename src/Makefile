CC     = gcc
CFLAGS = -std=gnu99 -Wall -Wextra -c
EXEC   = client image main

#################### MAIN ####################
main: main.o client.o config.o image.o position.o sensors.o tacho.o
	$(CC) main.o client.o config.o image.o position.o sensors.o tacho.o -o main -lm -lbluetooth -lpthread -lev3dev-c
	./main

main.o: main.c main.h const.h config.h tacho.h sensors.h position.h image.h client.h
	$(CC) -I./../../ev3dev-c/source/ev3 $(CFLAGS) main.c -o main.o

#################### CLIENT ####################
client: client.o
	$(CC) client.o -o client -lm -lbluetooth

client.o: client.c client.h const.h
	$(CC) $(CFLAGS) client.c -o client.o

#################### CONFIG ####################
config.o: config.h const.h
	$(CC) $(CFLAGS) config.c -o config.o

config.o_local: config.h const.h
	$(CC) -I./../../ev3dev-c/source/ev3 $CFLAGS config.c -o config.o

#################### IMAGE ####################
image: image.o client.o
	$(CC) image.o client.o -o image -lm -lbluetooth

image.o: image.c image.h const.h
	$(CC) $(CFLAGS) image.c -o image.o

#################### POSITION ####################
position.o: position.c position.h const.h client.h
	$(CC) $(CFLAGS) position.c -o position.o
	./image

#################### SENSORS ####################
sensors.o: sensors.c sensors.h
	$(CC) $(CFLAGS) sensors.c -o sensors.o

sensors.o_local: sensors.c sensors.h
	gcc -I./../../ev3dev-c/source/ev3 $CFLAGS sensors.c -o sensors.o

#################### TACHO ####################
tacho.o: tacho.c tacho.h const.h position.h
	$(CC) $(CFLAGS) tacho.c -o tacho.o

tacho.o_local: tacho.c tacho.h const.h position.h
	gcc -I./../../ev3dev-c/source/ev3 $CFLAGS tacho.c -o tacho.o

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
