// 시작 지점. 스위치로 주기.

#include "Arduino_BMI270_BMM150.h"
#include <ArduinoEigen.h>
#include <Servo.h>

using Eigen::Matrix;
using Eigen::Vector;

#define WAIT_TIME 12     // How often to run the code (in milliseconds)

float p[7] = {2.003259, 1.320377, 0.001008, 0.001081, -0.000776, 2.029001, 4.090002}; // 5.893260 0.920377 0.003478 0.002810 0.000128 2.929001 2.200000 
float dp[7] = {0.5, 0.5, 1e-4, 1e-4, 1e-4, 0.5, 0.5}; // 0.5 -> 1.0

Servo servo1;
int servo_angle = 90;  // 시작은 중앙
int off = 30;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

//float yaw_buffer[10] = {0};
//int yaw_index = 0;
//float yaw_sum = 0;
//float phi_filtered = 0;

float psi = 0.0;
float theta = 0.0;
float phi = 0.0;

float dt = 0.012;
unsigned long previousMillis = 0;

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

int servo_cnt;


void kalman_update() {
  // IMU 센서 읽기
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  float a_psi = atan2(ax, sqrt(ay * ay + az * az));
  float a_theta = atan2(ay, sqrt(ax * ax + az * az));

  psi = x(0); theta = x(1);
  float p = gx - x(2), q = gy - x(3), r = gz - x(4);
  float sec2 = 1.0 / pow(cos(theta), 2);

  A(0, 0) = 1 + dt * (q * cos(psi) * tan(theta) - r * sin(psi) * tan(theta));
  A(0, 1) =     dt * (q * sin(psi) * sec2 + r * cos(psi) * sec2);
  A(0, 2) = -dt;
  A(1, 0) =     dt * (-q * sin(psi) - r * cos(psi));
  A(1, 1) = 1;
  A(1, 3) = -dt;

  Vector<float, 5> xdot;
  xdot.setZero();
  xdot(0) = p + q * sin(psi) * tan(theta) + r * cos(psi) * tan(theta);
  xdot(1) = q * cos(psi) - r * sin(psi);

  Vector<float, 5> xp = x + xdot * dt;
  Matrix<float, 5, 5> Pp = A * P * A.transpose() + Q;

  Matrix<float, 5, 2> Ht = H.transpose();
  Matrix<float, 2, 2> S = H * Pp * Ht + R;
  Matrix<float, 5, 2> K = Pp * Ht * S.inverse();

  Vector<float, 2> z;
  z << a_psi - off_psi, a_theta - off_theta;

  Vector<float, 2> dz = z - H * xp;

  x = xp + K * dz;
  P = Pp - K * H * Pp;
}


float measure_kalman_error(float target_deg, float target_deg2) {

  int count = 0;
  unsigned long start = millis();
  Vector<float, 200> values;
  Vector<float, 200> values2;

  float target_rad = target_deg * PI / 180.0;
  float target_rad2 = target_deg2 * PI / 180.0;

  while (millis() - start < 2000) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      kalman_update();  // 기존 loop()의 필터 계산을 함수로 분리
      if (count < 200){
        values[count] = abs(x(0) - target_rad);
        values2[count] = abs(x(1) - target_rad2);
      }
      else{
        break;
      }
      count++;
    }
  }

  float sum = 0.0;
  float sum2 = 0.0;

  for (int i = 0; i < count; i++) {
    sum += values[i];
    sum2 += values2[i];
  }
  float mean_est = sum / count;
  float mean_est2 = sum2 / count;
  /*
  float var = 0.0;
  float var2 = 0.0;

  for (int i = 0; i < count; i++) {
    float diff = values[i] - mean_est;
    float diff2 = values2[i] - mean_est2;

    var += diff * diff;
    var2 += diff2 * diff2;
  }
  var /= count;
  var2 /= count;
  */
  
  return mean_est + mean_est2;  // 편향 + 진동 = 총 오차
}

float run_kalman_test(float p[7]) {
  // Q 설정
  Q(0, 0) = p[0] * var_gyro.x;
  Q(1, 1) = p[1] * var_gyro.y;
  Q(2, 2) = p[2];
  Q(3, 3) = p[3];
  Q(4, 4) = p[4];

  // R 설정
  R(0, 0) = p[5] * var_acc.x;
  R(1, 1) = p[6] * var_acc.y;

  // 테스트 시나리오: 0도 → 30도 → 0도
  servo1.write(90);  // 90 + 30
  //delay(500);
  float error1 = measure_kalman_error(0.0, 0.0);
  Serial.println("Start");
  Serial.print("Roll (deg): ");
  Serial.print(x(0) * 180.0 / PI);
  Serial.print("    Pitch (deg): ");
  Serial.println(x(1) * 180.0 / PI);
  servo1.write(120);  // 90 + 30
  delay(500);
  //float error2 = measure_kalman_error(00.0, 30.0);
  //Serial.print("Roll (deg): ");
  //Serial.print(x(0) * 180.0 / PI);
  //Serial.print("    Pitch (deg): ");
  //Serial.println(x(1) * 180.0 / PI);
  servo1.write(90);   // 원래대로
  delay(500);
  float error3 = measure_kalman_error(0.0, 0.0);
  Serial.print("Roll (deg): ");
  Serial.print(x(0) * 180.0 / PI);
  Serial.print("    Pitch (deg): ");
  Serial.println(x(1) * 180.0 / PI);

  return error1 + error3;
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


  // 측정 노이즈 공분산 R
  R.setZero();


  A.setZero();

  // 측정 행렬 H
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;


  servo_cnt = 0;


  float best_err = run_kalman_test(p);

  int iteration = 0;

  while ((dp[0]+dp[1]+dp[2]+dp[3]+dp[4]+dp[5]+dp[6]) > 0.01 && (iteration < 20)) {
    for (int i = 0; i < 7; i++) {
      p[i] += dp[i];
      float err = run_kalman_test(p);

      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];
        err = run_kalman_test(p);

        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }

    iteration++;
    Serial.print("Best Error: "); Serial.println(best_err,6);
    Serial.print("Current Params: ");
    for (int i = 0; i < 7; i++) Serial.print(p[i], 6), Serial.print(" ");
    Serial.println();
  }

}


void loop() {
}
