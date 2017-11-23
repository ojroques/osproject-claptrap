CC = gcc
CFLAGS = -Wall -c

client: client.o
	# $(CC) -Wall -lm -lev3dev-c -lbluetooth client.o -o client
	$(CC) -Wall -lm -lbluetooth client.o -o client
	./client

client.o: client.c client.h
	$(CC) -std=gnu99 -W $(CFLAGS) client.c -o client.o

image: image.o
	$(CC) image.o -o image
	./image

image.o: image.c image.h
	$(CC) $(CFLAGS) image.c -o image.o

clean:
	rm -f *.o
