CC     = gcc
CFLAGS = -Wall -c
EXEC   = client image main tacho sensors

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

tacho.o: tacho.c tacho.h
	$(CC) -std=gnu99 -W $(CFLAGS) tacho.c -o tacho.o

tacho: tacho.o
	$(CC) -Wall tacho.o -o tacho -lm -lev3dev-c
	./tacho

sensors.o: sensors.c sensors.h
	$(CC) -std=gnu99 -W $(CFLAGS) sensors.c -o sensors.o

sensors: sensors.o
	$(CC) -Wall sensors.o -o sensors -lm -lev3dev-c
	./sensors

# main: main.o client.o image.o tacho.o sensors.o
# 	$(CC) main.o image.o client.o tacho.o sensors.o -o main -lm -lbluetooth -lev3dev-c
# 	./main

main: main.o client.o image.o
	$(CC) main.o image.o client.o -o main -lm -lbluetooth
	./main

main.o: main.c main.h
	$(CC) -std=gnu99 -W $(CFLAGS) main.c -o main.o

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
