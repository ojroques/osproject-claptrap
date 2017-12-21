CC   = gcc
EXEC = sensors_test tester

tester: tester.c
	$(CC) -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	$(CC) tester.o -Wall -lm -lev3dev-c -o tester

sensors_test: sensors_test.c
	$(CC) -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors_test.c -o sensors_test.o
	$(CC) -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c ./../src/sensors.c -o sensors.o
	$(CC) sensors_test.o sensors.o -Wall -lm -lev3dev-c -o sensors_test

sensors_test_nathan: sensors_test.c
	$(CC) -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors_test.c -o sensors_test.o
	$(CC) -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c ./../src/sensors.c -o sensors.o
	$(CC) sensors_test.o sensors.o -Wall -L/home/nathan/workspace/Eurecom/OS_project/ev3dev-c-master/lib -lm -lev3dev-c -o sensors_test

run_tester:
	./tester

run_sensors_test:
	./sensors_test

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
