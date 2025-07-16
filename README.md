
IMU 센서에 내재된 자이로 센서(각속도 측정) + 가속도 센서(중력 가속도 측정(이동 가속도도 포함되어 있고 노이즈 심함))를 
Extended Kalman Filter를 사용해 센서 융합함.

즉, 단기적으로 정확한(bias 있어서 ㅠㅠ)로 예측하고 가속도 센서(bias 없음)로 장기적인 bias 보정.
또한, 상태변수에 bias를 넣어 추정해 더 안정적인 예측값 생성하도록 함.


상태변수 = [roll, pitch, biasx, biasy, biasz]


## result

<img width="281" height="125" alt="image" src="https://github.com/user-attachments/assets/e921d412-58ae-48f0-b219-ed467aa1ea06" />
