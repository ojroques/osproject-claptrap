tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/tester.c -o tests/tester.o
	gcc tests/tester.o -Wall -lm -lev3dev-c -o tests/tester

tacho_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/tacho_test.c -o test/tacho_test.o
	gcc tests/tacho_test.o -Wall -lm -lev3dev-c -o tests/tacho_test

movements:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c src/robot_movement/movements.c -o src/robot_movement/movements.o

test_main: movements
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/test_main.c -o tests/test_main.o
	gcc src/robot_movement/movements.o tests/test_main.o -Wall -lm -lev3dev-c -o tests/test_main

run_tester:
	./tester

run_tacho_test:
	./tacho_test
