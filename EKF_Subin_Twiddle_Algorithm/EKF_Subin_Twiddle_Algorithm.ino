// 시작 지점. 스위치로 주기.

#include "Arduino_BMI270_BMM150.h"
#include <ArduinoEigen.h>
#include <Servo.h>

using Eigen::Matrix;
using Eigen::Vector;

#define WAIT_TIME 12     // How often to run the code (in milliseconds)

double p[7] = {1.000000, 1.000000, 0.001000, 0.001000, 0.001000, 1.000000, 1.000000 }; //0.888100 0.999100 0.000100 0.000100 0.000100 0.999100 0.999100 
double dp[7] = {p[0]/10, p[1]/10, p[2]/10, p[3]/10, p[4]/10, p[5]/10, p[6]/10}; 
/*
0.001915  0.002145  0.000033  0.000036
Current Params: 0.800000 0.100000 0.001000 0.001000 0.001000 90.800000 9.400000 
*/
Servo servo1;
int servo_angle = 90;  // 시작은 중앙
int off = 30;

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

sensor mean_acc[500] = {};
sensor mean_gyro[500] = {};

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


double measure_kalman_error(double target_deg, double target_deg2) {

  int count = 0;

  Vector<double, 500> values;
  Vector<double, 500> values2;


  while (count < 500) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      kalman_update();  // 기존 loop()의 필터 계산을 함수로 분리
      //Serial.print(x(0)* 180.0 / PI);
      //Serial.print("    ");
      //Serial.println(x(1)* 180.0 / PI);
      values[count] = abs((x(0) * 180.0 / PI) - target_deg);
      values2[count] = abs((x(1) * 180.0 / PI) - target_deg2);
      count++;
      delay(WAIT_TIME);
    }
  }

  double sum = 0.0;
  double sum2 = 0.0;

  for (int i = 0; i < count; i++) {
    sum += values[i];
    sum2 += values2[i];
  }

  // bias 오차
  double mean_est = sum / count;
  double mean_est2 = sum2 / count;

  sum = 0.0;
  sum2 = 0.0;

  for(int i = 0; i < count; i++){
    sum += pow((values[i] - mean_est),2);
    sum2 += pow((values2[i] - mean_est2),2);
  }
  // 분산 오차
  sum /= (count - 1);
  sum2 /= (count - 1);

  return (sum + sum2);  // 편향 + 진동 = 총 오차
}

double run_kalman_test(double p[7]) {
  // Q 설정
  Q(0, 0) = p[0] * var_gyro.x;
  Q(1, 1) = p[1] * var_gyro.y;
  Q(2, 2) = p[2];
  Q(3, 3) = p[3];
  Q(4, 4) = p[4];

  // R 설정
  R(0, 0) = p[5] * var_acc.x;
  R(1, 1) = p[6] * var_acc.y;

  if(Q(0,0) < 0.000001){
    Q(0,0) = 0.000001;
  }
  if(Q(1,1) < 0.000001){
    Q(1,1) = 0.000001;
  }
  if(R(0,0) < 0.000001){
    R(0,0) = 0.000001;
  }
  if(R(1,1) < 0.000001){
    R(1,1) = 0.000001;
  }

  int count = 0;
  while (count < 300) { // 초기 안정화용
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      kalman_update();
      count++;
      delay(WAIT_TIME);
    }
  }

  Serial.println("var_gyro and var_acc");
  Serial.print(var_gyro.x,6);
  Serial.print("  ");
  Serial.print(var_gyro.y,6);
  Serial.print("  ");
  Serial.print(var_acc.x,6);
  Serial.print("  ");
  Serial.println(var_acc.y,6);

  Serial.println("P value");
  for(int i = 0; i < 7; i++){
    Serial.print(p[i],6);
    Serial.print("  ");
  }
  Serial.println();
  Serial.println("Q and R value");
  Serial.print(Q(0,0),6);
  Serial.print("  ");
  Serial.print(Q(1,1),6);
  Serial.print("  ");
  Serial.print(R(0,0),6);
  Serial.print("  ");
  Serial.println(R(1,1),6);


  // 테스트 시나리오: 0도 → 30도 → 0도
  servo1.write(90);  // 90 + 30
  double error1 = measure_kalman_error(0.0, 0.0);
  Serial.print("error:  ");
  Serial.println(error1,6);

  Serial.println();
  Serial.println();
  delay(5000);
  return error1;
}

void kalman_reset(){
  
  // 상태 변수 x = [0 0 0 0 0]'
  x.setZero();
  x(2) = sum_gyro.x;
  x(3) = sum_gyro.y;
  x(4) = sum_gyro.z;

  // 공분산 행렬 P = diag([1 1 0.01 0.01 0.01])
  P.setZero();
  P(0, 0) = 1,0;
  P(1, 1) = 1,0;
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
  while(cnt < 500){
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
  

  for (int i = 0; i < 500; i++) {
    sum_acc.x += mean_acc[i].x;
    sum_acc.y += mean_acc[i].y;
    sum_acc.z += mean_acc[i].z;

    sum_gyro.x += mean_gyro[i].x;
    sum_gyro.y += mean_gyro[i].y;
    sum_gyro.z += mean_gyro[i].z;
  }

  sum_acc.x = sum_acc.x / 500;
  sum_acc.y = sum_acc.y / 500;
  sum_acc.z = sum_acc.z / 500;

  sum_gyro.x = sum_gyro.x / 500;
  sum_gyro.y = sum_gyro.y / 500;
  sum_gyro.z = sum_gyro.z / 500;

  for (int i = 0; i < 500; i++){
    var_acc.x += (mean_acc[i].x - sum_acc.x) * (mean_acc[i].x - sum_acc.x);
    var_acc.y += (mean_acc[i].y - sum_acc.y) * (mean_acc[i].y - sum_acc.y);
    var_acc.z += (mean_acc[i].z - sum_acc.z) * (mean_acc[i].z - sum_acc.z);
    var_gyro.x += (mean_gyro[i].x - sum_gyro.x) * (mean_gyro[i].x - sum_gyro.x);
    var_gyro.y += (mean_gyro[i].y - sum_gyro.y) * (mean_gyro[i].y - sum_gyro.y);
    var_gyro.z += (mean_gyro[i].z - sum_gyro.z) * (mean_gyro[i].z - sum_gyro.z);
  }

  var_acc.x = var_acc.x / 499;
  var_acc.y = var_acc.y / 499;
  var_acc.z = var_acc.z / 499;

  var_gyro.x = var_gyro.x / 499;
  var_gyro.y = var_gyro.y / 499;
  var_gyro.z = var_gyro.z / 499;

  if(var_acc.x < 0.00001){
    var_acc.x = 0.00001;
  }
  if(var_acc.y < 0.00001){
    var_acc.y = 0.00001;
  }
  if(var_acc.z < 0.00001){
    var_acc.z = 0.00001;
  }
  if(var_gyro.x < 0.00001){
    var_gyro.x = 0.00001;
  }
  if(var_gyro.y < 0.00001){
    var_gyro.y = 0.00001;
  }
  if(var_gyro.z < 0.00001){
    var_gyro.z = 0.00001;
  }
  

  Serial.println("Fixed var_acc and var_gyro and sum_gyro:");
  Serial.print("var_acc.x: "); Serial.println(var_acc.x, 6);
  Serial.print("var_acc.y: "); Serial.println(var_acc.y, 6);
  Serial.print("var_gyro.x: "); Serial.println(var_gyro.x, 6);
  Serial.print("var_gyro.y: "); Serial.println(var_gyro.y, 6);
  Serial.print("sum_gyro.x: "); Serial.println(sum_gyro.x, 6);
  Serial.print("sum_gyro.y: "); Serial.println(sum_gyro.y, 6);
  Serial.print("sum_gyro.z: "); Serial.println(sum_gyro.z, 6);
  Serial.print("sum_acc.x: "); Serial.println(sum_acc.x, 6);
  Serial.print("sum_acc.y: "); Serial.println(sum_acc.y, 6);
  Serial.print("sum_acc.z: "); Serial.println(sum_acc.z, 6);
  delay(5000);

  off_psi = atan2(sum_acc.x, sqrt(sum_acc.y * sum_acc.y + sum_acc.z * sum_acc.z));
  off_theta = atan2(sum_acc.y, sqrt(sum_acc.x * sum_acc.x + sum_acc.z * sum_acc.z));

  //off_psi = atan2(sum_acc.y, sum_acc.z));
  //off_theta = atan2(sum_acc.x, sqrt(sum_acc.y*sum_acc.y + sum_acc.z*sum_acc.z));
  
  
  
  servo_cnt = 0;

  double best_err;

  int iteration = 0;

  while ((dp[0]+dp[1]+dp[2]+dp[3]+dp[4]+dp[5]+dp[6]) > 0.01 && (iteration < 100)) {
    int idx = -1;
    int up[7] = {0,0,0,0,0,0,0};

    kalman_reset();
    best_err = run_kalman_test(p);
    Serial.print("Best Error: "); Serial.println(best_err,6);

    for (int i = 0; i < 7; i++) {
      if(i == 2){
        i = 5;
      }
      p[i] += dp[i];
      kalman_reset();
      double err_plus = run_kalman_test(p);
      double err_minus = 10000.0;
      delay(100);
      // -dp[i] 방향 테스트
      p[i] -= 2 * dp[i];  // 이제 -dp[i] 위치
      
      if(p[i] > 0){
        kalman_reset();
        err_minus = run_kalman_test(p);
        delay(100);
      }
      p[i] += dp[i];
      
      // 둘 중 더 좋은 방향 선택
      if (err_plus < err_minus && err_plus < best_err) {
        best_err = err_plus;
        idx = i;
        up[i] = 1;
      } 
      else if (err_minus < err_plus && err_minus < best_err) {
      // 그대로 -dp[i] 유지
        best_err = err_minus;
        idx = i;
      }     
      
    }

    if(idx != -1){
      if(up[idx] == 1){
        p[idx] += dp[idx];
        Serial.print("change idx  ");
        Serial.println(idx);
        delay(2000);
      }
      else{
        p[idx] -= dp[idx];
        Serial.print("change idx  ");
        Serial.println(idx);
        delay(2000);
      }
      //dp[idx] /= 2;

      iteration++;
      Serial.print("Best Error: "); Serial.println(best_err,6);
      Serial.print("Current Params: ");
      for (int i = 0; i < 7; i++) Serial.print(p[i], 6), Serial.print(" ");
      Serial.println();
    }
    else{
      Serial.println("Change parameter dp");
      for(int i = 0; i < 7; i++){
        if(i == 2){
        i = 5;
        }
        dp[i] *= 2;
        //dp[i] /= 2;
      }
      Serial.print("Best Error: "); Serial.println(best_err,6);
      Serial.print("Current Params: ");
      for (int i = 0; i < 7; i++) Serial.print(p[i], 6), Serial.print(" ");
    }
    
  }

}


void loop() {
}