#include "Module.h"
#include "RF24.h"
#include "RF24_config.h"
#include "mbed.h"
#include "nRF24L01.h"
#include "variables.h"

#define D2R 0.01745329251994327
#define R2D 57.2957795130823799
#define ROWS 3
#define COLS 4

PwmOut PWM1(p21);
PwmOut PWM2(p22);
PwmOut PWM3(p23);
PwmOut PWM4(p24);
DigitalOut led1(p26);
DigitalOut led_switch(LED4);
DigitalIn switch_on(p5, PullDown);
Ticker loops;

I2C i2c(p28, p27); //(I2C_SDA,I2C_SCL); //sda, scl I2C_SDA,I2C_SCL D4, D5 A4, A5
Serial pc(USBTX, USBRX); // D1D0
RF24 NRF24L01(p5, p6, p7, p15, p8);
Timer t;
// RF
const uint64_t pipe = 0x1212121212LL;


void control(void);
void pwm_drive(void);
void control_loop(void);
void RF_READ();
void RF_init(int nchannel);
int constrain_int16(int16_t x, int min, int max);
void calib(void);
void arm(void);
void RF_READ();
void RF_init(int nchannel);
void euler_bias_cal(void);
int constrain_int16(int16_t x, int min, int max);

// Quaternion
void cal_F(float * SE_q, float *E_d, float *S_s, float *row);
void updateQuternion(float *q ,float *gyro, float dt);
void quaternProd (float *row,const float *a, const float *b);
void quaternConj(float *q, float *q_conj, float *row);
void correctQuaternion(float * q, float *accel);
void transpose(float src[ROWS][COLS], float dest[COLS][ROWS]);
void matmul(float mat1[COLS][ROWS], float mat2[ROWS][1], float result[COLS][1]);
void normalize(float vector[COLS][1]);



int but1;
int but1_prev = 1;
int but2;
int but2_prev = 1;
bool bias_flag=true;

float thrust_prev=0;
float roll_ref_prev=0;
float pitch_ref_prev=0;
float yaw_ref_prev=0;
float thrust_in_prev;
bool star=true;
//float max_rate_change = 2.0;
float bias_r;
float bias_p;
float bias_y;

int main() {
  // Start calibration
  RF_init(60); // channel 60

  pc.baud(115200);
  PWM1.period(0.0001);
  PWM2.period(0.0001);
  PWM3.period(0.0001);
  PWM4.period(0.0001); // 10 kHz PWM for PWM1~PWM4
  pc.baud(115200);

  WHO_AM_I();

  MPU9250_INIT();

  t.start();
  while (1) {

    if (l_count >= 2) {
      ThisThread::sleep_for(20);
      RF_READ();
      roll_ref = (float)ROLL*0.12/Kc_scale; // -100 ~100 -->30 0.12
      pitch_ref = (float)PITCH*0.12/Kc_scale;
      yaw_ref = (float)YAW*0.12/Kc_scale;
      thrust_in = (float)THROTTLE*0.8;  //-100~100
      but1 = int(BUT1);
      but2 = int(BUT2);

      led1 = 1;
      //pc.printf("waiting\n\r");
      //pc.printf("BUTTON: %d, %d, %d, %d\n\r", but1, but2, but1_prev,but2_prev);
      //pc.printf("ROLL: %f, PITCH: %f, YAW: %f, THROTTLE: %f, BUT1:%d, BUT2: %d \n\r", roll_ref, pitch_ref, yaw_ref, thrust_in, but1, but2); 
      if (but1 == 0 && but1_prev == 1) {
        calib();
        bias_flag=true;
      }
      if (but2 == 0 && but2_prev == 1) {
        arm();
      }
      but1_prev = but1;
      but2_prev = but2;

      l_count = 0;
    }
    if (arm_flag == true && t.read_us()>5000) {
        
        control_loop();
        //pc.printf("THROTTLE: %f, %f\n\r", thrust_prev, thrust_in);
        //pc.printf("ROLL: %f, PITCH: %f, YAW: %f, THROTTLE: %f, BUT1:%d, BUT2: %d \n\r", roll_ref, pitch_ref, yaw_ref, thrust_in, but1, but2); 
        t.reset();
      }
      
       //control_loop();
      // pc.printf("%f, %f, %f, %f, %d, %d \n\r", pitch_ref, roll_ref, yaw_ref,
      // thrust_in, but1, but2); //pc.printf("%d\n\r", THROTTLE); pc.printf("I'm
      // working\n\r");
      if(thrust_in!=0){
          thrust_prev += thrust_in/400;
          if((thrust_in/400)>0.075){
              thrust_prev+= 1.0/60.0;
          }
      }
      l_count = l_count + 1;
    }
  }

  // loops.attach_us(&control_loop,2500);

  // measurement
  // find error
  // find control output
  //  pwm dirve

void control_loop(void) {
  int16_t gyro[3];
  int16_t accel[3];
  float roll_accel_1;
  float pitch_accel_1;
  float accel_fu[3];
  static float accel_tau = 2;
  static  float accel_f1[3];
  static float accel_f[3] = {0, 0, 0};
  static float accel_f2[3] ; // added


  MPU9250_GET_GYRO(gyro);
  gyro_f[0] = gyro[0] / 32.8 - gyro_bias[0]; //deg per sec
  gyro_f[1] = -(gyro[1] / 32.8 - gyro_bias[1]); //32.8
  gyro_f[2] = -(gyro[2] / 32.8 - gyro_bias[2]);

  gyro_angle[0] = gyro_angle[0] + (gyro_f[0]) * 0.02; //deg 
  gyro_angle[1] = gyro_angle[1] + (gyro_f[1]) * 0.02;
  gyro_angle[2] = gyro_angle[2] + (gyro_f[2]) * 0.02;

  MPU9250_GET_ACCEL(accel);
  accel_f[0] = accel[0] / 8192.0 ; //%Navigation frame reference (NED) unit in G (9.8 m/sec^2)
  accel_f[1] = accel[1] / 8192.0 * (-1);
  accel_f[2] = accel[2] / 8192.0 * (-1);

  roll_accel_1 = atan2(-accel_f[1], -accel_f[2]) * 180.0 / 3.14; // rad to deg
  pitch_accel_1 = atan2(accel_f[0], sqrt(accel_f[1] * accel_f[1] +accel_f[2] * accel_f[2])) *180.0 / 3.14;


  if (roll_accel_1 < 85 && roll_accel_1> -85)
    roll_accel = roll_accel_1;
  if (pitch_accel_1 < 85 && pitch_accel_1 > -85)
    pitch_accel = pitch_accel_1;

  //상보
  sinPhi = sin(roll_f*D2R), cosPhi = cos(roll_f*D2R); //rad
  cosTheta = cos(pitch_f*D2R), tanTheta = tan(pitch_f*D2R);

  phi_rate = gyro_f[0]+ sinPhi*tanTheta*gyro_f[1] +cosPhi*tanTheta*gyro_f[2] ;//- euler_bias[0]; //deg per sec
  theta_rate = cosPhi*gyro_f[1] - sinPhi*gyro_f[2]*D2R ;//- euler_bias[1] ;
  psi_rate = sinPhi/cosTheta*gyro_f[1] +cosPhi/cosTheta*gyro_f[2] ;//- euler_bias[1];
 

  float alpha = 0.90;
  roll_f= alpha*(roll_f+ 0.02*phi_rate)+(1-alpha)*roll_accel ; //deg
  pitch_f = alpha*(pitch_f + 0.02*theta_rate) + (1-alpha)*pitch_accel +0.2;
  yaw_f = yaw_f + 0.02*psi_rate; //dt*gyro_f[2]; //

//   // 칼만 필터
//   phi_rate = gyro_f[0] + sinPhi * tanTheta * gyro_f[1] + cosPhi * tanTheta * gyro_f[2] ; // deg per sec
//   theta_rate = cosPhi * gyro_f[1] - sinPhi * gyro_f[2] * D2R ;
//   psi_rate = sinPhi / cosTheta * gyro_f[1] + cosPhi / cosTheta * gyro_f[2];
  
//   phi_rate_pred = phi_rate_pred + phi_rate*dt;
//   theta_rate_pred = theta_rate_pred + theta_rate*dt;
//   psi_rate_pred = psi_rate_pred +psi_rate*dt;


  if(star==true){
      bias_r=roll_f;
      bias_p=pitch_f;
      //bias_y=yaw_f;
      //pc.printf("BIAS: %f, %f\n\r", bias_r, bias_p);
      star=false;
  }

  //pc.printf("비교 : gyro_f: %f, phi_f: %f, diff: %f \n\r" ,gyro_f[0], phi_rate, (gyro_f[0]-phi_rate));
  //pc.printf("비교 / gyro_R : %f, gyro_P: %f  EULER_R: %f , EULER_P: %f, \n\r", roll_g_n, pitch_g_n, roll_f_n, pitch_f_n );
  //pc.printf("비교: body_R:%f, euler_R: %f, diff: %f  \n\r",dt*gyro_f[0], dt*phi_rate, dt*(gyro[0]-phi_rate) );
  //pc.printf("상보필터 결과 : ROLL: %f, PITCH: %f, YAW: %f \n\r " ,roll_f-bias_r, pitch_f-bias_p, yaw_f);


  control();

  pwm_drive();
  

}

void control(void) {
   //조종기 인풋 기울기 제한 ---->넘 느려                            
//   roll_ref = (fabs(roll_ref - roll_ref_prev) > max_rate_change)? roll_ref_prev +max_rate_change * ((roll_ref > roll_ref_prev) ? 1 : -1): roll_ref;
//   pitch_ref = (fabs(pitch_ref - pitch_ref_prev) > max_rate_change)? pitch_ref_prev + max_rate_change *((pitch_ref > pitch_ref_prev) ? 1 : -1): pitch_ref;
//   yaw_ref =
//       (fabs(yaw_ref - yaw_ref_prev) > max_rate_change)? yaw_ref_prev + max_rate_change * ((yaw_ref > yaw_ref_prev) ? 1 : -1): yaw_ref;
  //thrust_in = (fabs(thrust_in - thrust_in_prev) > max_rate_change)? thrust_in_prev + max_rate_change *((thrust_in > thrust_in_prev) ? 1 : -1): thrust_in;
  // Update previous values
//   roll_ref_prev = roll_ref;
//   pitch_ref_prev = pitch_ref;
//   yaw_ref_prev = yaw_ref;
  //thrust_in_prev = thrust_in;

  roll_err = roll_ref - roll_f + bias_r; //각도 에러 (roll_err) //deg
  roll_rate_err = 0 - phi_rate ; // deg per sec  (roll_ref-roll_ref_prev)/dt
  pitch_err = -pitch_ref - pitch_f + bias_p; // (pitch_err)
  pitch_rate_err = 0 - theta_rate; //(pitch_ref - pitch_ref_prev)/dt
  yaw_err = yaw_ref - yaw_f;
  yaw_rate_err = 0 -  psi_rate; //(yaw_ref-yaw_ref_prev)/dt


  
  
//   // for 적분기
roll_err_int += roll_err * dt;
pitch_err_int += pitch_err * dt;

  if (roll_err > ROLL_ERR_MAX)
    roll_err = ROLL_ERR_MAX;
  else if (roll_err < -(ROLL_ERR_MAX))
    roll_err = -(ROLL_ERR_MAX);
  if (roll_rate_err > ROLL_RATE_ERR_MAX)
    roll_rate_err = ROLL_RATE_ERR_MAX;
  else if (roll_rate_err < -(ROLL_RATE_ERR_MAX))
    roll_rate_err = -(ROLL_RATE_ERR_MAX);

  roll_cont = K_scale * (Kp_roll * roll_err + Kd_roll * roll_rate_err +Ki_roll * roll_err_int); // pid

  if (roll_cont > 50.0)
    roll_cont = 50.0;
  else if (roll_cont < -50.0)
    roll_cont = -50.0;

  roll_c = roll_cont/100 ;
  // pitch control
  if (pitch_err > PITCH_ERR_MAX) {
    pitch_err = PITCH_ERR_MAX;
  } else if (pitch_err < -(PITCH_ERR_MAX)) {
    pitch_err = -(PITCH_ERR_MAX);
  }

  if (pitch_rate_err > PITCH_RATE_ERR_MAX) {
    pitch_rate_err = PITCH_RATE_ERR_MAX;
  } else if (pitch_rate_err < -(PITCH_RATE_ERR_MAX)) {
    pitch_rate_err = -(PITCH_RATE_ERR_MAX);
  }
  // this is PROBLEM
  pitch_cont = K_scale * (Kp_pitch * pitch_err + Kd_pitch * pitch_rate_err + Ki_pitch * pitch_err_int);

  if (pitch_cont > 50.0) {
    pitch_cont = 50.0;
  } else if (pitch_cont < -50.0) {
    pitch_cont = -50.0;
  }

  pitch_c = pitch_cont / 100  ;

  // yaw control
  yaw_c = K_scale * (Kp_yaw * yaw_err + Kd_yaw * yaw_rate_err) / 100 ;

  if(thrust_prev>80){thrust_prev=80;}
  if(thrust_prev<-80){thrust_prev=-80;}
  thrust_c = thrust_prev/80; //(thrust_in-80)/160;
  
  //pc.printf("Roll: %f, Pitch: %f, Yaw:%f, Throttle:%f \n\r" , roll_err, pitch_err, yaw_err, thrust_c);
  //pc.printf("\rRoll: %f, Pitch: %f, Yaw:%f, Throttle:%f " , roll_c, pitch_c, yaw_c, thrust_c); // roll <--> pitch
  
  
}

void pwm_drive(void) {
  float cont_v=0;
  float cont_m=0;

  pwm1_pw = (-roll_c - pitch_c - yaw_c + thrust_c); // 수정 필요
  pwm2_pw = (roll_c - pitch_c + yaw_c + thrust_c);
  pwm3_pw = (roll_c + pitch_c - yaw_c + thrust_c);
  pwm4_pw = (-roll_c + pitch_c + yaw_c + thrust_c);

  float max_pwm = pwm1_pw;
  if (pwm2_pw > max_pwm)
    max_pwm = pwm2_pw;
  if (pwm3_pw > max_pwm)
    max_pwm = pwm3_pw;
  if (pwm4_pw > max_pwm)
    max_pwm = pwm4_pw;

  if( max_pwm>=1.0){
    cont_v= max_pwm - 1.0;
}

    PWM1 = pwm1_pw -cont_v ;//+ cont_m;
    PWM2 = pwm2_pw -cont_v;//+ cont_m;
    PWM3 = pwm3_pw -cont_v;//+ cont_m;
    PWM4 = pwm4_pw -cont_v;//+ cont_m;

  pc.printf("\n\r1: %f, 2: %f, 3:%f, 4:%f 5: %f", pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw, cont_v);
}

void RF_READ(){
     if (NRF24L01.available())   
     {
        NRF24L01.read(recv, 10); 

        ROLL  = *(int16_t*)(&recv[0]); //ROLL = - ROLL; 
        PITCH = *(int16_t*)(&recv[2]); //flip pitch and roll
        PITCH = PITCH;
        YAW = *(int16_t*)(&recv[4]);
        THROTTLE = *(int16_t*)(&recv[6]); 
        BUT1 = recv[8];
        BUT2 = recv[9]; //should hold value here
        
        //pc.printf("%d, %d, %d, %d \n\r", PITCH, ROLL, YAW, (THROTTLE)); //pc.printf("%d\n\r", THROTTLE);        
        
        rf_fail_count = 0;
      }
      
      else {
          //pc.printf(" DATA IS UNAVAILABLE");
          rf_fail_count = rf_fail_count + 1;
          
            if(rf_fail_count >= 20 && rf_fail_count < 100){
                THROTTLE = THROTTLE - 2;
                THROTTLE = constrain_int16(THROTTLE, 0, 1023);
            }
          
                if(rf_fail_count >= 100){
                THROTTLE = 0;
                
            }
          if(rf_fail_count >= 100) {
            rf_fail_count = 100;
          }
    }
}

void RF_init(int nchannel)
{
    
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(nchannel);  // set channel 10 20 30
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2,4); //1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
    pc.printf("CHANNEL");
   }
    
int constrain_int16(int16_t x, int min, int max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}


void calib(void) {
  gyro_angle[0] = 0.0;
  gyro_angle[1] = 0.0;
  gyro_angle[2] = 0.0;

  roll_f=0;
  pitch_f=0;
  yaw_f=0;

  PWM1 = 0;
  PWM2 = 0;
  PWM3 = 0;
  PWM4 = 0;

  star=true;

  roll_err_int = 0;
  pitch_err_int = 0;

  thrust_prev=0;
  
  pc.printf("CALIBRATION\n\r");
  gyro_bias_f();
  pc.printf("gyro biases(deg/sec) %f %f %f \n\r", gyro_bias[0], gyro_bias[1],
            gyro_bias[2]);
  led1 = 0;
  pc.printf("let's go\n\r");
  wait(3);
}

void arm(void) {
  if (arm_flag == false) {
    arm_flag = true;
  } else {
    pc.printf("I'm dead \n\r");
    
    PWM1 = 0;
    PWM2 = 0;
    PWM3 = 0;
    PWM4 = 0;
    arm_flag = false;
  }
}

// void euler_bias_cal(void){
//     static int16_t gyro_e[3];

//     for (int i = 0; i < 100; i++) {
//     MPU9250_GET_GYRO(gyro_e);
//     gyro_f[0] = gyro_e[0] / 32.8 - gyro_bias[0]; //deg per sec
//     gyro_f[1] = -(gyro_e[1] / 32.8 - gyro_bias[1]); //32.8
//     gyro_f[2] = -(gyro_e[2] / 32.8 - gyro_bias[2]);
//     sinPhi = sin(roll_f_n*D2R), cosPhi = cos(roll_f_n*D2R); //rad
//     cosTheta = cos(pitch_f_n*D2R), tanTheta = tan(pitch_f_n*D2R);

//     phi_rate = gyro_f[0] + sinPhi*tanTheta*gyro_f[1] +cosPhi*tanTheta*gyro_f[2]; //deg per sec
//     theta_rate = cosPhi*gyro_f[1] - sinPhi*gyro_f[2];
//     psi_rate = sinPhi/cosTheta*gyro_f[1] +cosPhi/cosTheta*gyro_f[2];
    
//     euler_bias[0] = euler_bias[0] + phi_rate;
//     euler_bias[1] = euler_bias[1] + theta_rate;
//     euler_bias[2] = euler_bias[1] + psi_rate;

//     //pc.printf("EULER_bias finding i= %d\n\r", i);
//   }
//   euler_bias[0] = euler_bias[0]/100;
//   euler_bias[1] = euler_bias[1]/100;
//   euler_bias[2] = euler_bias[1] /100;



//   pc.printf("Euler bias finding completed %\n\r");
//   pc.printf("euler biases(deg/sec) %f %f %f \n\r", euler_bias[0], euler_bias[1], euler_bias[2]);
// }

void updateQuternion(float *q ,float *gyro, float dt){
    float gx = gyro[0]*D2R;
    float gy = gyro[1]*D2R;
    float gz = gyro[2]*D2R;

    float q_dot[4]={
        0.5f*(-q[1]*gx - q[2]*gy - q[3]*gz),
        0.5f*(q[0]*gx + q[2]*gz -q[3]*gy),
        0.5f*(q[0]*gy - q[1]*gz + q[3]*gz),
        0.5f*(q[0]*gz + q[1]*gy - q[2]*gz)
    };

    // intergrate and update quaternion
    q[0] += q_dot[0]*dt;
    q[1] += q_dot[1]*dt;
    q[2] += q_dot[2]*dt;
    q[3] += q_dot[3]*dt;

    // Normalize quaternion to maintain validity 
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
    q[4] /= norm;
 }

// quaternion calculation
 void quaternProd (float *row,const float *a, const float *b){
     
     float a1=a[0];
     float a2=a[1];
     float a3=a[2];
     float a4=a[3];

     float b1=b[0];
     float b2=b[1];
     float b3=b[2];
     float b4=b[3];

     row[0] = a1*b1 - a2*b2 - a3*b3 - a4*b4;
     row[1] = a1*b2 + a2*b1 +a3*b4 -a4*b3;
     row[2] = a1*b3 - a2*b4 + a3*b1 +a4*b2;
     row[3] = a1*b4 + a2*b3 -a3*b2 +a4*b1;

 }


 void quaternConj(float *q, float *q_conj, float *row){
     //float q_conj[4]; ---> 이것도 외부 선언
     q_conj[0] = q[0];
     q_conj[1] = q[1];
     q_conj[2] = q[2];
     q_conj[3] = q[3];
 }


 void correctQuaternion(float * q, float *accel){
     float accel_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
     float g_n[3] ={0.0, 0.0, 0.0};
     float E_d[3] = {g_n[0], g_n[1], g_n[2]};
     float S_s[3] = {accel[0]/accel_norm, accel[1]/accel_norm, accel[2]/accel_norm};

     float q_t[3] = {q[0],q[1],q[2]};

     float row_j[3][4];
     float row[3][1];

     //SE_q :quaternion 1*4
     // E_d : 1*4 reference 물리량[0,dx,dt,dz]
     //S_s: 1*4 측정 물리량 [0,sx,sy,sz]

    // Reference frame measurement vector
     float dx = E_d[0];
     float dy = E_d[1];
     float dz = E_d[2];

     //Sensor frame measured vector
     float sx = S_s[0];
     float sy = S_s[1];
     float sz = S_s[2];

     // Orientation quatsernion vector
     float q1 = q[0]; //SE_q
     float q2 = q[1];
     float q3 = q[2];
     float q4 = q[3];
    //궅이 2차원 배열로?
     row[0][0] = 2.0f*dx*(0.5f-q3*q3 - q4*q4) + 2.0f*dy*(q1*q4 +q2*q3) + 2.0f *dz*(q2*q4 -q1*q3) -sx;
     row[1][0]= 2.0f*dx*(q2*q3 -q1*q4) +2.0f*dy*(0.5f -q2*q2-q4*q4) + 2.0f*dz*(q1*q2 -q3*q4) -sy;
     row[2][0] = 2.0f*dx*(q1*q3 +q2*q4) + 2.0f*dt*(q3*q4-q1*q2) +2.0f*dz*(0.5 -q2*q2-q3*q3)-sz;

     row_j[0][0]= 2.0f*dy*q4 -2.0f*dz*q3;
     row_j[0][1] = 2.0f*dy*q3 +2.0*dz*q4;
     row_j[0][2] = -4.0*dx*q3 +2.0f*dy*q2 -2.0*dz*q1;
     row_j[0][3] = -4.0*dx*q4 + 2.0f*dy*q1 + 2.0f*dz*q1 +2.0f*dz*q2;

     row_j[1][0] = -2.0f*dx*q4 -2.0f*dy*q2;
     row_j[1][1] = 2.0f*dx*q3 - 4*dy*q2 + 2*dz*q1;
     row_j[1][2] = 2*dx*q2 + 2.0f*dz*q4;
     row_j[1][3] = 2.0f*dx*q1 - 4.0f*dy*q4 + 2*dz*q3; ;

     row_j[2][0] = 2.0f*dx*q3 - 2.0f*dy*q2;
     row_j[2][1] = 2*dx*q4 -2*dy*q1 -4.0f*dz*q2;
     row_j[2][2] = 2.0f*dx*q1 +2*dy*q4 - 4.0f*dz*q3;
     row_j[2][3] = 2*dx*q2 +2*dy*q3;

     float DeltaT[COLS][ROWS];
     float step[COLS][1];

     // traspose delta
     transpose(row_j, DeltaT);

     // multiply delatT and F
     matmul(DeltaT, row, step);

     //normilize(step)
     normalize(step);


    float error[3]={
        step[0][1],
        step[1][1],
        step[2][1]
    };

     float correction[3] = {error[0], error[1], error[2]};

     float gyro_corrected[3] = {
         gyro_f[0] + correction[0],
         gyro_f[1] + correction[1],
         gyro_f[2] + correction[2]

     };

     updateQuternion(q, gyro_corrected,dt);
 }

 void updateOrientation(float  *gyro, float *accel, float *q, float dt){
     //update quaternion with gyro data
     updateQuternion(q, gyro, dt);

     //update quaternion using accel data
     correctQuaternion(q, accel);
 }

 void transpose(float src[ROWS][COLS], float dest[COLS][ROWS]) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            dest[j][i] = src[i][j];
        }
    }
}

void matmul(float mat1[COLS][ROWS], float mat2[ROWS][1], float result[COLS][1]) {
    for (int i = 0; i < COLS; i++) {
        result[i][0] = 0.0f;
        for (int j = 0; j < ROWS; j++) {
            result[i][0] += mat1[i][j] * mat2[j][0];
        }
    }
}

void normalize(float vector[COLS][1]) {
    float norm = 0.0f;

    // Calculate the norm (Euclidean)
    for (int i = 0; i < COLS; i++) {
        norm += vector[i][0] * vector[i][0];
    }
    norm = sqrtf(norm);

    // Normalize the vector
    if (norm > 0.0f) {
        for (int i = 0; i < COLS; i++) {
            vector[i][0] /= norm;
        }
    }
}