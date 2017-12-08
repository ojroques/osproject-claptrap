CC     = gcc
CFLAGS = -std=gnu99 -Wall -Wextra -c
EXEC   = client image main

# main: main.o client.o image.o tacho.o sensors.o
# 	$(CC) main.o image.o client.o tacho.o sensors.o -o main -lm -lbluetooth -lev3dev-c
# 	./main

main: main.o
	$(CC) main.o -o main
	./main

main.o: main.c main.h
	$(CC) $(CFLAGS) main.c -o main.o

client: client.o
	$(CC) client.o -o client -lm -lbluetooth
	./client

client.o: client.c client.h
	$(CC) $(CFLAGS) client.c -o client.o

image: image.o client.o
	$(CC) image.o client.o -o image -lm -lbluetooth
	./image

image.o: image.c image.h
	$(CC) $(CFLAGS) image.c -o image.o

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
