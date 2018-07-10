#include <stdio.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <gps.h>
#include <wiringPi.h>
#include "motor.h"
#include "mitibiki.h"
#include "ring_buffer.h"
#include "pid.h"
#include "acclgyro.h"
#include "xbee_at.h"



static const int GPS_RING_LEN = 10;//gps�̃����O�o�b�t�@�̒���
static const double STACK_THRESHOLD = 0.000001; //stack���肷��Ƃ���臒l
static const int GOAL_THRESHOLD = 3;
static const int SETPOINT = 0.0;//delta_angle�̖ڕW�l
static const double KP_VALUE = 0.65;
static const double KI_VALUE = 0.00005;
static const double KD_VALUE = 0;
static const int PID_LEN = 20;
static const int MAX_STACK_ROTATE_TIMES = 20;

float PGAIN = 2.0;//�������@����m�F
float IGAIN = 0.01;

float GOAL_LAT;
float GOAL_LON;

float DestLat, DestLon, OriginLat, OriginLon;// �ŐV�̈ܓx�A�ŐV�̌o�x�A���݂̈ܓx�A���݂̌o�x
float rundata[4];
int i = 1;



//�V�O�i���n���h��
void handler(int signum)
{
	motor_stop();
	delay(100);
	exit(1);
}

int printTime()
{
	time_t timer;
	time(&timer);
	printf("%s\n", ctime(&timer));
	return 0;
}


//DistAngle�̍X�V
int updateDistAngle(DistAngle *data, Queue* latring, Queue* lonring)
{
	data->dist2goal = dist_on_sphere(getLast(latring), getLast(lonring));
	data->angle2goal = calc_target_angle(getLast(latring), getLast(lonring));
	printf("angle2goal:%f\n", data->angle2goal);
	return 0;
}

int updateCoord(Queue* latring, Queue* lonring)
{
	loc_t coord;
	gps_location(&coord);//gps�f�[�^�擾
	if (coord.latitude == 0.0)
	{
		printf("GPS return 0 value\n");
		//RING BUFFER�̍X�V�͂��Ȃ�(stack�����쓮�̂���)
	}
	else
	{
		enqueue(latring, coord.latitude); //�ܓx���i�[
		enqueue(lonring, coord.longitude); //�o�x���i�[
		printf("time:%f\nlatitude:%f\nlongitude:%f\n", coord.time, coord.latitude, coord.longitude);
		xbeePrintf("latitude:%f\r\nlongitude:%f\r\n", coord.latitude, coord.longitude);
	}
	return 0;
}

//gps�ƒn���C�̃f�[�^����񕪍X�V���A�����O�o�b�t�@�Ɋi�[
int updateAll(DistAngle* data, Queue* latring, Queue* lonring)
{
	printTime();
	updateCoord(latring, lonring);
	updateDistAngle(data, latring, lonring);
	//GPS_RING_LEN�܂Ń����O�o�b�t�@�����܂��������琏�����I��stack ����
	if (queue_length(latring) == GPS_RING_LEN)
	{
		stackJudge(latring, lonring);
	}
	return 0;
}



double courseTo(double lat1, double long1, double lat2, double long2)
{
	// returns course in degrees (North=0, West=270) from position 1 to position 2,
	// both specified as signed decimal-degrees latitude and longitude.
	// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
	// Courtesy of Maarten Lamers
	double dlon = radians(long2 - long1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	double a1 = sin(dlon) * cos(lat2);
	double a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += 6.283185307;
	}
	return degrees(a2);
}


float PIDcontrol(float command, float current)  //command �ڕW�n�@current�@���ݒn
{
	float controlValue;
	float error;
	static float i_error = 0.0;

	error = command - current;
	i_error += error;

	controlValue = PGAIN * error + IGAIN * i_error; //��ᐧ��@
													
	return (controlValue);
}

float AngleNormalization(float angle) //���E�̔��ʂ����₷�����邽�߂̊p�x�̒�`�̏C��
{
	if (angle >= 180)
		angle = angle - 360;
	else if (angle <= -180)
		angle = angle + 360;

	return (angle);
}




int main()
{
	signal(SIGINT, handler);
	printTime();
	pwm_initialize();
	gps_init();
	xbee_init();
	DistAngle DistAngle_data = { 0,0,100000,0 };
	Queue* gps_latring = make_queue(GPS_RING_LEN);
	Queue* gps_lonring = make_queue(GPS_RING_LEN);

	OriginLat = coord.latitude;
	OriginLon = coord.longitude;

	rundata[0] = OriginLat;
	rundata[1] = OriginLon;

	updateAll(* data, * latring, * lonring);
	
	distance = (unsigned long)corseTo(
		OriginLat,
		OriginLon,
		GOAL_LAT,
		GOAL_LON);

	rundata[2] = distance;
	rundata[3] = 0;
	transferData(rundata, 4);
	if (distance <= GOAL_RANGE)
	{
		printf("goal");
		motor_control(0, 0);
		while (1);
	}

	motor_control(75, 75);
	delay(10000);
	motor_control(0, 0);
	while (1)
	{
		float x, y, z;
		float originLat, originLon;
		float angle1, angle2, angle3;
		static float angle = 0;
		float dt;
		float controlValue;
		float error;
		unsigned long distance;

		Pid pid_data;
		pid_initialize(&pid_data);
		pid_const_initialize(&pid_data, SETPOINT, KP_VALUE, KI_VALUE, KD_VALUE);
		int i;

		loc_t coord;
		gps_location(&coord);//gps�f�[�^�擾
		if (coord.latitude == 0.0)
		{
			printf("GPS return 0 value\n");
			//RING BUFFER�̍X�V�͂��Ȃ�(stack�����쓮�̂���)
		}
		else
		{
			enqueue(latring, coord.latitude); //�ܓx���i�[
			enqueue(lonring, coord.longitude); //�o�x���i�[
			printf("time:%f\nlatitude:%f\nlongitude:%f\n", coord.time, coord.latitude, coord.longitude);
			xbeePrintf("latitude:%f\r\nlongitude:%f\r\n", coord.latitude, coord.longitude);
		}
		//measure_gyro(&x, &y, &z);
		//angle += z * dt;
		//angle = DEG2RAD*AngleNormalization(angle);
		angle = 0;
		gelay(2000);

		DestLat = coord.latitude;
		DestLon = coord.longitude;
		
		rundata[0] = DestLat;
		rundata[1] = DestLon;

		dist2goal = dist_on_sphere(getLast(latring), getLast(lonring));

		distance = dist2goal;


		if (distance <= GOAL_RANGE)
		{
			printf("goal");
			motor_control(0, 0);
			while (1);
		}

		angle1 = courseTo(
			OriginLat, OriginLon, DestLat, DestLon);
		angle2 = angle2goal;

		angle3 = angle1 - angle2;

		rundata[3] = angle3;

		error = angle3 - angle;

		controlValue = PIDcontrol(0, error);
		controlValue = constrain(controlValue, -77, 77);
		motor_control(75 - controlValue, 75 + controlValue);
		OriginLat = DestLat;
		OriginLon = DestLon;

		pid_data.input = -(data->delta_angle);
		compute_output(&pid_data);
		printf("pid_output = %d\n", pid_data.output);
	}
	}
}