# Makefile
LIBS = -lwiringPi -lm
objs_f = flight_2.o bme280.o xbee_at.o acclgyro.o #flight


flight_2.out: $(objs_f)
	gcc -g -Wall -O2 -o flight_2.out $(objs_f) $(LIBS)


flight_2.o: flight_2.c
	gcc -c -Wall flight_2.c


bme280.o:bme280.h bme280.c
	gcc -c -Wall bme280.c

xbee_at.o:xbee_at.h xbee_at.c
	gcc -c -Wall xbee_at.c

acclgyro.o:acclgyro.h acclgyro.c
	gcc -c -Wall acclgyro.c




