// 시작 지점. 스위치로 주기.

#include "Arduino_BMI270_BMM150.h"
#include <ArduinoEigen.h>
#include <Servo.h>

using Eigen::Matrix;
using Eigen::Vector;

#define WAIT_TIME 12     // How often to run the code (in milliseconds)

Servo servo1;
int servo_angle = 90;  // 시작은 중앙
int off = 30;
int ccnt = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;


double psi = 0.0;
double theta = 0.0;
double phi = 0.0;

double dt = 0.012;
unsigned long previousMillis = 0;

typedef struct{
  double x;
  double y;
  double z;
} sensor;

sensor mean_acc[300] = {};
sensor mean_gyro[300] = {};


sensor var_acc = {};
sensor var_gyro = {};

sensor sum_acc = {};
sensor sum_gyro = {};


// 칼만필터 시스템 변수 설정.
Vector<double, 5> x;
Matrix<double, 5, 5> P, Q, A;
Matrix<double, 2, 5> H;
Matrix<double, 2, 2> R;

double off_psi;
double off_theta;

int servo_cnt;


void kalman_update() {
  // IMU 센서 읽기
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  ax *= 9.81;
  ay *= 9.81;
  az *= 9.81;

  double a_psi = atan2(ax, sqrt(ay * ay + az * az));
  double a_theta = atan2(ay, sqrt(ax * ax + az * az));

  psi = x(0); theta = x(1);
  double p = gx - x(2), q = gy - x(3), r = gz - x(4);
  double sec2 = 1.0 / pow(cos(theta), 2);

  A(0, 0) = 1 + dt * (q * cos(psi) * tan(theta) - r * sin(psi) * tan(theta));
  A(0, 1) =     dt * (q * sin(psi) * sec2 + r * cos(psi) * sec2);
  A(0, 2) = -dt;
  A(1, 0) =     dt * (-q * sin(psi) - r * cos(psi));
  A(1, 1) = 1;
  A(1, 3) = -dt;

  Vector<double, 5> xdot;
  xdot.setZero();
  xdot(0) = p + q * sin(psi) * tan(theta) + r * cos(psi) * tan(theta);
  xdot(1) = q * cos(psi) - r * sin(psi);

  Vector<double, 5> xp = x + xdot * dt;
  Matrix<double, 5, 5> Pp = A * P * A.transpose() + Q;

  Matrix<double, 5, 2> Ht = H.transpose();
  Matrix<double, 2, 2> S = H * Pp * Ht + R;
  Matrix<double, 5, 2> K = Pp * Ht * S.inverse();

  Vector<double, 2> z;
  z << a_psi - off_psi, a_theta - off_theta;

  Vector<double, 2> dz = z - H * xp;

  x = xp + K * dz;
  P = Pp - K * H * Pp;
}





void kalman_reset(){
  
  // 상태 변수 x = [0 0 0 0 0]'
  x.setZero();
  x(2) = sum_gyro.x;
  x(3) = sum_gyro.y;
  x(4) = sum_gyro.z;

  // 공분산 행렬 P = diag([1 1 0.01 0.01 0.01])
  P.setZero();
  P(0, 0) = 1.0;
  P(1, 1) = 1.0;
  P(2, 2) = 0.1;
  P(3, 3) = 0.1;
  P(4, 4) = 0.1;

  // 프로세스 노이즈 공분산 Q
  Q.setZero();


  // 측정 노이즈 공분산 R
  R.setZero();

  Q(0, 0) = 0.001915 * 0.800000;
  Q(1, 1) = 0.002145 * 0.100000;
  Q(2, 2) = 0.001;
  Q(3, 3) = 0.001;
  Q(4, 4) = 0.001;

  // R 설정
  R(0, 0) = 0.000033 * 90.800000;
  R(1, 1) = 0.000036 * 9.400000;

  A.setZero();

  // 측정 행렬 H
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  
}


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  servo1.attach(11); // PWM 핀 (ex: D11)
  servo1.write(servo_angle);

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");

  Serial.print("Magnetic Field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println("Hz");
  int cnt = 0;
  while(cnt < 300){
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      ax *= 9.81;
      ay *= 9.81;
      az *= 9.81;
      mean_acc[cnt] = {ax,ay,az};
      mean_gyro[cnt] = {gx, gy, gz};

      cnt++;
      delay(WAIT_TIME);
    }
  }
  

  for (int i = 0; i < 300; i++) {
    sum_acc.x += mean_acc[i].x;
    sum_acc.y += mean_acc[i].y;
    sum_acc.z += mean_acc[i].z;

    sum_gyro.x += mean_gyro[i].x;
    sum_gyro.y += mean_gyro[i].y;
    sum_gyro.z += mean_gyro[i].z;
  }

  sum_acc.x = sum_acc.x / 300;
  sum_acc.y = sum_acc.y / 300;
  sum_acc.z = sum_acc.z / 300;

  sum_gyro.x = sum_gyro.x / 300;
  sum_gyro.y = sum_gyro.y / 300;
  sum_gyro.z = sum_gyro.z / 300;

  
  off_psi = atan2(sum_acc.x, sqrt(sum_acc.y * sum_acc.y + sum_acc.z * sum_acc.z));
  off_theta = atan2(sum_acc.y, sqrt(sum_acc.x * sum_acc.x + sum_acc.z * sum_acc.z));

  kalman_reset();
}


void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      kalman_update();
      delay(WAIT_TIME);
      Serial.print(x(0) * 180.0 / PI);
      Serial.print(",");
      Serial.println(x(1) * 180.0 / PI);
  }

}