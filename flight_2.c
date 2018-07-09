#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <wiringPi.h>
#include "bme280.h"
#include "xbee_at.h"
#include "acclgyro.h"

#define GPIO17 17
#define GPIO27 27
int main(void){
	bme280_initialize();
	xbee_init();
	Accl accl_data;
	acclGyro_initialize();
	int cut=1;
	double altitude;
	
	pinMode(GPIO17,OUTPUT);
	pinMode(GPIO27,OUTPUT);

	while(1){
		altitude=readAltitude();
		printf("x_accl is %f\n",accl_data.acclX_scaled);
		printf("y_accl is %f\n",accl_data.acclY_scaled);
		printf("z_accl is %f\n",accl_data.acclZ_scaled);


		printf("%f\n",altitude);
		xbeePrintf("altitude%f\r\n",altitude);

		digitalWrite(GPIO17,1);
		digitalWrite(GPIO27,1);
		if(altitude>=20&&cut==1){
			cut=2;
			printf("%d\n",cut);
		}
		else if(altitude<=5&&cut==2){
			cut=3;
			printf("%d\n",cut);
		}
		else if(altitude<=1&&cut==3){
			cut=4;
			printf("%d\n",cut);
		}
		else if(cut==4){
			printf("release");
			digitalWrite(GPIO17,0);
			digitalWrite(GPIO27,0);
			delay(10000);
			break;
		}

	}
	xbee_close();
	return 0;
}
	










