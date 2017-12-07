CC     = gcc
CFLAGS = -Wall -c
EXEC   = client image main

client: client.o
	$(CC) -Wall client.o -o client -lm -lbluetooth
	./client

client.o: client.c client.h
	$(CC) -std=gnu99 -W $(CFLAGS) client.c -o client.o

image: image.o client.o
	$(CC) image.o client.o -o image -lm -lbluetooth
	./image

image.o: image.c image.h
	$(CC) -std=gnu99 -W $(CFLAGS) image.c -o image.o

# main: main.o client.o image.o tacho.o sensors.o
# 	$(CC) main.o image.o client.o tacho.o sensors.o -o main -lm -lbluetooth -lev3dev-c
# 	./main

sensors.o:
		gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors.c -o sensors.o

main: main.o
	$(CC) main.o -o main
	./main

main.o: main.c main.h
	$(CC) -std=gnu99 -W $(CFLAGS) main.c -o main.o

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
