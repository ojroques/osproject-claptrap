tacho_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tacho_test.c -o tacho_test.o
	gcc tacho_test.o -Wall -lm -lev3dev-c -o tacho_test

color_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c color_test.c -o color_test.o
	gcc color_test.o -Wall -lm -lev3dev-c -o color_test

gyro_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c gyro_test.c -o gyro_test.o
	gcc gyro_test.o -Wall -lm -lev3dev-c -o gyro_test

all_test:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c all_test.c -o all_test.o
	gcc all_test.o -Wall -lm -lev3dev-c -o all_test

run_tacho_test:
	./tacho_test

run_color_test:
	./color_test

run_gyro_test:
	./gyro_test

run_all_test:
	./all_test

clean:
	rm *.o
