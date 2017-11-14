tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -o tester

tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tacho_test.c -o tacho_test.o
	gcc tacho_test.o -Wall -lm -lev3dev-c -o tacho_test

run_tester:
	./tester

run_tacho_test:
	./tacho_test
