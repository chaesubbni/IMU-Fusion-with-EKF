
IMU 센서에 내재된 자이로 센서(각속도 측정) + 가속도 센서(중력 가속도 측정(이동 가속도도 포함되어 있고 노이즈 심함))를 
Extended Kalman Filter를 사용해 센서 융합함.

즉, 단기적으로 정확한(bias 있어서 ㅠㅠ)로 예측하고 가속도 센서(bias 없음)로 장기적인 bias 보정.
또한, 상태변수에 bias를 넣어 추정해 더 안정적인 예측값 생성하도록 함.


상태변수 = [roll, pitch, biasx, biasy, biasz]


## result

<img width="281" height="86" alt="image" src="https://github.com/user-attachments/assets/bcb20b75-300e-4bad-ab98-033aae97d64b" />

하지만 사실 지금은 EKF의 Q와 R 값을 자이로 각속도 x,y 분산값 + 내 임의로 선정한 값임.
그렇다보니 아직 최적의 결과는 아님.

그래서 최근에 PID 제어를 공부 중 Twiddle 방식이 있음.
Twiddle 알고리즘은 PID 제어나 칼만 필터 등의 파라미터 최적화에 자주 사용되는 방식으로, 초기값을 기준으로 오차를 측정한 뒤, 성능이 개선되면 해당 파라미터를 더 크게(×1.1) 조정하고, 그렇지 않으면 반대 방향(-2×)으로 시도하는 방식임. 
만약 반대 방향에서도 성능 향상이 없다면 조정 폭을 줄여(×0.9) 탐색 범위를 좁혀 나감.
결국 각 파라미터마다 최적값을 찾기 위해 ±k 범위를 반복적으로 탐색하며 오차를 최소화해가는 알고리즘.


## Twiddle test

시스템의 Q와 R을 보다 정밀하게 설정하기 위해, 정지 상태에서의 센서 노이즈, 움직인 후에 발생하는 동적 노이즈, 그리고 다시 원래 위치로 복귀했을 때의 편향(bias)을 반영한 노이즈까지 모두 고려해 총 오차(error)를 계산함.
이러한 방식은 단순한 정적 오차 분석보다 실제 동작 환경을 반영한 종합적인 오차 측정이 가능하므로, 보다 현실적이고 신뢰도 높은 Q와 R 설정에 유리하다고 판단함.

<img width="605" height="277" alt="image" src="https://github.com/user-attachments/assets/f5ac0517-e38d-415e-9ba6-ceb837ede1b7" />

<img width="581" height="249" alt="image" src="https://github.com/user-attachments/assets/290372ec-1f88-4e8e-b6fe-da02e7c51af3" />


## Final test

<img width="301" height="290" alt="image" src="https://github.com/user-attachments/assets/1e023041-f3ed-4c0f-988b-a762bb11f9b8" />


## Graph

영상 보고싶으면 이미지 클릭해주세요!!

[![Watch the video](https://img.youtube.com/vi/PG9V5bQvgVE/0.jpg)](https://youtu.be/PG9V5bQvgVE)


