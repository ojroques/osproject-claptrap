EXEC = tacho_test sensors_test gyro_test tester

tester: tester.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -o tester

tacho_test: tacho_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tacho_test.c -o tacho_test.o
	gcc tacho_test.o -Wall -lm -lev3dev-c -o tacho_test

sensors_test: sensors_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors_test.c -o sensors_test.o
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c ./../src/sensors.c -o sensors.o
	gcc sensors_test.o sensors.o -Wall -lm -lev3dev-c -o sensors_test

sensors_test_nathan: sensors_test.c
	gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors_test.c -o sensors_test.o
	gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c ./../src/sensors.c -o sensors.o
	gcc sensors_test.o sensors.o -Wall -L/home/nathan/workspace/Eurecom/OS_project/ev3dev-c-master/lib -lm -lev3dev-c -o sensors_test

gyro_test: gyro_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c gyro_test.c -o gyro_test.o
	gcc gyro_test.o -Wall -lm -lev3dev-c -o gyro_test

run_tester:
	./tester

run_tacho_test:
	./tacho_test

run_sensors_test:
	./sensors_test

run_gyro_test:
	./gyro_test

clean:
	rm -f *.o

ultraclean:
	rm -f *.o $(EXEC)
