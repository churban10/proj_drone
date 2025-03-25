#ifndef __VARIABLES_H__
#define __VARIABLES_H__
#include "mbed.h"

extern int8_t recv[30];
extern int16_t PITCH , ROLL , YAW , THROTTLE;
extern int8_t ackData[30];
extern int8_t flip1;
// ETC
extern float roll_controller, pitch_controller, yaw_controller ;
extern char BUT1, BUT2, prev_BUT2;
extern int rf_fail_count ;

extern float gyro_bias[3] ;

// taken from MPU9250 library
extern int l_count;

//  float dt=0.0025; //0.0025
extern float dt;
extern float accel_f[3], gyro_f[3];
extern float gyro_angle[3];
extern float roll_f, roll_accel, roll_g_n;
extern float roll_g_old ;
extern float roll_a_n;
extern float roll_a_old;
extern float pitch_f, pitch_accel, pitch_g_n;
extern float pitch_g_old ;
extern float pitch_a_n;
extern float pitch_a_old ;
extern float yaw_f ;
extern float tau ;

extern float K_scale;
//  float Kp_roll=1.2, Kd_roll=0.8, Ki_roll=0;
//  float Kp_pitch=1.5, Kd_pitch=0.4, Ki_pitch=0;  // p=50 0.02 ,  p=100 0.06,
//  p>100 0.15
extern float Kp_roll , Kd_roll , Ki_roll ;
extern float Kp_pitch , Kd_pitch ,Ki_pitch ; // p=50 0.02 ,  p=100 0.06,  p>100 0.15
extern float Kp_yaw , Kd_yaw ;
extern float Kc_scale;

extern float roll_cont , pitch_cont;
extern float roll_c , pitch_c , yaw_c , thrust_c ;
extern float roll_err ;
extern float roll_err_int ;
extern float roll_rate_err ;
extern float roll_ref , roll_rate_ref ;
extern float pitch_err ;
extern float pitch_err_int;
extern float pitch_rate_err ;
extern float pitch_ref , pitch_rate_ref ;
extern float yaw_err;
extern float yaw_rate_err ;
extern float yaw_ref , yaw_rate_ref ;
extern float thrust_in ;

extern float pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw;
extern float ROLL_ERR_MAX,
      ROLL_RATE_ERR_MAX ; // maximum errors considered
extern float PITCH_ERR_MAX ,
      PITCH_RATE_ERR_MAX ; // maximum errors considered
extern float roll_p;

extern float sinPhi, cosPhi;
extern float cosTheta, tanTheta;
extern float phi_rate, theta_rate, psi_rate;
extern bool arm_flag;
extern float alpha;
//test
extern float roll_f_n;
extern float roll_f_old;
extern float pitch_f_n;
extern float pitch_f_old;
extern float euler_bias[3];

#endif
