#include "Arduino_BMI270_BMM150.h"
#include <ArduinoEigen.h>

using Eigen::Matrix;
using Eigen::Vector;

#define WAIT_TIME 12     // How often to run the code (in milliseconds)


float ax, ay, az;
float gx, gy, gz;
int accel_x = 0;
int accel_y = 0;
unsigned long previousMillis = 0;
float dt = 0.05;

typedef struct{
  float x;
  float y;
  float z;
} sensor;


sensor mean_acc[300] = {};
sensor mean_gyro[300] = {};


sensor var_acc = {};
sensor var_gyro = {};


// 칼만필터 시스템 변수 설정.
Vector<float, 5> x;
Matrix<float, 5, 5> P, Q, A;
Matrix<float, 2, 5> H;
Matrix<float, 2, 2> R;

float off_psi;
float off_theta;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");

  int cnt = 0;
  while(cnt < 300){
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      mean_acc[cnt] = {ax,ay,az};
      mean_gyro[cnt] = {gx, gy, gz};

      cnt++;
    }
    
  }
  
  sensor sum_acc = {};
  sensor sum_gyro = {};

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

  for (int i = 0; i < 300; i++){
    var_acc.x += (mean_acc[i].x - sum_acc.x) * (mean_acc[i].x - sum_acc.x);
    var_acc.y += (mean_acc[i].y - sum_acc.y) * (mean_acc[i].y - sum_acc.y);
    var_acc.z += (mean_acc[i].z - sum_acc.z) * (mean_acc[i].z - sum_acc.z);
    var_gyro.x += (mean_gyro[i].x - sum_gyro.x) * (mean_gyro[i].x - sum_gyro.x);
    var_gyro.y += (mean_gyro[i].y - sum_gyro.y) * (mean_gyro[i].y - sum_gyro.y);
    var_gyro.z += (mean_gyro[i].z - sum_gyro.z) * (mean_gyro[i].z - sum_gyro.z);
  }

  var_acc.x = var_acc.x / 299;
  var_acc.y = var_acc.y / 299;
  var_acc.z = var_acc.z / 299;

  var_gyro.x = var_gyro.x / 299;
  var_gyro.y = var_gyro.y / 299;
  var_gyro.z = var_gyro.z / 299;

  off_psi = atan2(sum_acc.x, sqrt(sum_acc.y * sum_acc.y + sum_acc.z * sum_acc.z));
  off_theta = atan2(sum_acc.y, sqrt(sum_acc.x * sum_acc.x + sum_acc.z * sum_acc.z));

  //off_psi = atan2(sum_acc.y, sum_acc.z));
  //off_theta = atan2(sum_acc.x, sqrt(sum_acc.y*sum_acc.y + sum_acc.z*sum_acc.z));
  
  // 상태 변수 x = [0 0 0 0 0]'
  x.setZero();

  // 공분산 행렬 P = diag([1 1 0.01 0.01 0.01])
  P.setZero();
  P(0, 0) = 1.0;
  P(1, 1) = 1.0;
  P(2, 2) = 0.1;
  P(3, 3) = 0.1;
  P(4, 4) = 0.1;

  // 프로세스 노이즈 공분산 Q
  Q.setZero();
  Q(0, 0) = var_gyro.x;
  Q(1, 1) = var_gyro.y;
  Q(2, 2) = 1e-4;
  Q(3, 3) = 1e-4;
  Q(4, 4) = 1e-4;

  // 측정 노이즈 공분산 R
  R.setZero();
  R(0, 0) = 1.2*var_acc.x;
  R(1, 1) = 1.2*var_acc.y;

  A.setZero();

  // 측정 행렬 H
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;


}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= WAIT_TIME){
    previousMillis = currentMillis;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);


      float a_psi = atan2(ax, sqrt(ay * ay + az * az));  //  * 180 / PI
      float a_theta  = atan2(ay, sqrt(ax * ax + az * az)); //  * 180 / PI
      
      //float a_psi = atan2(ay/az);  //  * 180 / PI
      //float a_theta  = atan2(ax / sqrt(ay * ay + az * az)); //  * 180 / PI

      float phi = x(0);
      float theta = x(1);

      float p = gx - x(2);
      float q = gy - x(3);
      float r = gz - x(4);

      float sec2 = 1.0f / pow(cos(theta), 2);
      A(0, 0) = 1 + dt * (q * cos(phi) * tan(theta) - r * sin(phi) * tan(theta));
      A(0, 1) =     dt * (q * sin(phi) * sec2 + r * cos(phi) * sec2);
      A(0, 2) = -dt;
      A(1, 0) =     dt * (-q * sin(phi) - r * cos(phi));
      A(1, 1) = 1;
      A(1, 3) = -dt;


      Vector<float, 5> xdot(5);
      xdot.setZero();

      xdot(0) = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
      xdot(1) = q * cos(phi) - r * sin(phi);

      // 예측 단계
      Vector<float, 5> xp = x + xdot * dt;
      Matrix<float, 5, 5> Pp = A * P * A.transpose() + Q;

      // 칼만 이득 계산
      Matrix<float, 5, 2> Ht = H.transpose();
      Matrix<float, 2, 2> S = H * Pp * Ht + R;
      Matrix<float, 5, 2> K = Pp * Ht * S.inverse();

      // 측정값과 예측값 차이
      Vector<float, 2> z;
      z << a_psi - off_psi, a_theta - off_theta;

      Vector<float, 2> dz = z - H * xp;

      // 상태 업데이트
      x = xp + K * dz;

      // 공분산 업데이트
      P = Pp - K * H * Pp;

      Serial.print("Roll (deg): ");
      Serial.print(x(0) * 180.0 / PI);
      Serial.print("    Pitch (deg): ");
      Serial.println(x(1) * 180.0 / PI);
    }
       
  }
  
}