CC     = gcc
CFLAGS = -Wall -c
EXEC   = client image

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

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
