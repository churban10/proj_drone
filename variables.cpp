#include "variables.h"


int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int8_t ackData[30];
int8_t flip1 = 1;
// ETC
float roll_controller, pitch_controller, yaw_controller = 0.0;
char BUT1, BUT2, prev_BUT2;
int rf_fail_count = 0;

float gyro_bias[3] = {0.0, 0.0, 0.0};
float euler_bias[3]={0.0,0.0,0.0};

// taken from MPU9250 library
int l_count = 0;

//  float dt=0.0025; //0.0025
float dt=0.0025;
float accel_f[3]={0.0,0.0,0.0}, gyro_f[3]={0.0,0.0,0.0};
float gyro_angle[3]={0.0,0.0,0.0};
float roll_f=0;
float roll_accel, roll_g_n;
float roll_g_old = 0;
float roll_a_n;
float roll_a_old = 0;
float pitch_f, pitch_accel, pitch_g_n;
float pitch_g_old = 0;
float pitch_a_n;
float pitch_a_old = 0;
float yaw_f = 0;
float tau = 0.1;
// test
float roll_f_n;
float roll_f_old=0;
float pitch_f_n;
float pitch_f_old=0;
//**********************************************************
// Gain value
float K_scale = 1.0;
float Kp_roll = 0.80, Kd_roll = 0.20, Ki_roll = 0.15;//0.3; //0.75 0.15 0.01  0.8 0.2 0.0
float Kp_pitch = 0.80, Kd_pitch = 0.20, Ki_pitch = 0.15;//0.3; // p=50 0.02 ,  p=100 0.06,  p>100 0.15
float Kp_yaw = 0.80, Kd_yaw = 0.20;
float Kc_scale=1.0;

float roll_cont = 0.0, pitch_cont;
float roll_c = 0.0, pitch_c = 0.0, yaw_c = 0.0, thrust_c = 0.0;
float roll_err = 0;
float roll_err_int = 0;
float roll_rate_err = 0;
float roll_ref = 0.0, roll_rate_ref = 0.0;
float pitch_err = 0;
float pitch_err_int = 0;
float pitch_rate_err = 0;
float pitch_ref = 0.0, pitch_rate_ref = 0.0;
float yaw_err;
float yaw_rate_err = 0;
float yaw_ref = 0.0, yaw_rate_ref = 0.0;
float thrust_in = 0.0;

float pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw;
float ROLL_ERR_MAX = 50.0,
      ROLL_RATE_ERR_MAX = 50.0; // maximum errors considered
float PITCH_ERR_MAX = 50.0,
      PITCH_RATE_ERR_MAX = 50.0; // maximum errors considered
float roll_p;

float sinPhi, cosPhi;
float cosTheta, tanTheta;
float phi_rate, theta_rate, psi_rate;
bool arm_flag;
float alpha;