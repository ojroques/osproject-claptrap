tacho_test: tacho_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tacho_test.c -o tacho_test.o
	gcc tacho_test.o -Wall -lm -lev3dev-c -o tacho_test

color_test: color_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c color_test.c -o color_test.o
	gcc color_test.o -Wall -lm -lev3dev-c -o color_test

gyro_test: gyro_test.c
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c gyro_test.c -o gyro_test.o
	gcc gyro_test.o -Wall -lm -lev3dev-c -o gyro_test

robotclient: robotclient.c
	gcc -std=gnu99 -W -Wall -c robotclient.c -o robotclient.o
	gcc robotclient.o -Wall -lm -lev3dev-c -lbluetooth -o robotclient

run_tacho_test:
	./tacho_test

run_color_test:
	./color_test

run_gyro_test:
	./gyro_test

clean:
	rm *.o
