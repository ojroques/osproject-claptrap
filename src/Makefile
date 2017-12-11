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

# main: main.o client.o image.o tacho.o sensors.o
# 	$(CC) main.o image.o client.o tacho.o sensors.o -o main -lm -lbluetooth -lev3dev-c
# 	./main

sensors.o:
	$(CC) $(CFLAGS) sensors.c -o sensors.o

sensors.o_local:
		gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors.c -o sensors.o

config.o:
	$(CC) $(CFLAGS) config.c -o config.o

main: main.o
	$(CC) main.o -o main
	./main

main.o: main.c main.h
	$(CC) $(CFLAGS) main.c -o main.o

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
