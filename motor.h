#ifndef MOTOR_H
#define MOTOR_H
//C++から呼び出すため
#ifdef __cplusplus
extern "C" {
#endif

	int pwm_initialize();
	void motor_control(int L, int R)
#ifdef __cplusplus
}
#endif

#endif
