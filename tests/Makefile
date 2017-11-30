tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -o tester

tacho_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tacho_test.c -o tacho_test.o
	gcc tacho_test.o -Wall -lm -lev3dev-c -o tacho_test

color_test:
	gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c color_test.c -o color_test.o
	gcc -I./../../ev3dev-c-master/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c sensors.c -o sensors.o
	gcc color_test.o sensors.o -Wall -L/home/nathan/workspace/Eurecom/OS_project/ev3dev-c-master/lib -lm -lev3dev-c -o color_test

run_tester:
	./tester

run_tacho_test:
	./tacho_test

run_color_test:
	./color_test
