tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/tester.c -o tests/tester.o
	gcc tests/tester.o -Wall -lm -lev3dev-c -o tests/tester

tacho_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/tacho_test.c -o test/tacho_test.o
	gcc tests/tacho_test.o -Wall -lm -lev3dev-c -o tests/tacho_test

tacho.o:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c src/tacho.c -o src/tacho.o

position.o:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c src/position.c -o src/position.o

test_main: tacho.o position.o
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tests/test_main.c -o tests/test_main.o
	gcc src/position.o src/tacho.o tests/test_main.o -Wall -lm -lpthread -lev3dev-c -o tests/test_main

run_tester:
	./tester

run_tacho_test:
	./tacho_test
