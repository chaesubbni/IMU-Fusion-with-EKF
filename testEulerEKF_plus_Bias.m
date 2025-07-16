%상태변수 - roll pitch biasx biasy biasz

clear all
delete(instrfindall); % 혹시 시리얼이 다른 곳에서 열려 있으면 종료

Nsamples = 2000;
EulerSaved = zeros(Nsamples, 3);

dt = 0.01;

[a,g,m,soft,hard] = calcurate_sensor_noise(dt);

accel_var = var(a);
gyro_var = var(g);
mag_var = var(m);

a_mean = mean(a);

%가속도 센서값으로 phi, theta 구하기
phi_off = atan(a_mean(2)/a_mean(3));
theta_off = atan(a_mean(1)/sqrt(a_mean(2)^2 + a_mean(3)^2));

disp("Finish noise calculation");
pause(1);

%칼만필터 시스템 변수 설정
x = [0 0 0 0 0]';
P = diag([1 1 1e-1 1e-1 1e-1]);
Q = diag([gyro_var(1) gyro_var(2) 1e-5 1e-5 1e-5]);
R = 1.2 * diag([accel_var(1) accel_var(2)]);
A = zeros(5,5);
H = [1 0 0 0 0;
     0 1 0 0 0];

%이동 평균 필터 사용하기 위해
window_size = 10;
psi_buffer = zeros(window_size, 1);

%인터넷에서 자기장 편차 찾은 값.
Offset_mag = -0.1559;

for k = 1:Nsamples
    [accel, gyro, mag] = imu_read();

    %psi부터 구할게용가리
    mag = (mag - hard)*soft;
    psi = atan2(mag(2), mag(1));
    psi = psi + Offset_mag;

    if psi < 0
        psi = psi + 2*pi;
    end
    if psi > 2*pi
        psi = psi - 2*pi;
    end

    psi_buffer = [psi_buffer(2:end); psi];
    psi_filterd = mean(psi_buffer);
    
    %가속도 센서로 roll, pitch 구하기 - 보정할려고.
    a_phi = atan(accel(2)/accel(3));
    a_theta = atan(accel(1)/sqrt(accel(2)^2 + accel(3)^2));

    a_phi = a_phi - phi_off;
    a_theta = a_theta - theta_off;
    
    % 비선형 함수를 선형화 시켜 A 구함.
    phi = x(1);
    theta = x(2);

    p = gyro(1) - x(3);
    q = gyro(2) - x(4);
    r = gyro(3) - x(5);

    A(1,1) = 1 + dt * (q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta));
    A(1,2) =     dt * (q*sin(phi)*sec(theta)^2 + r*cos(phi)*sec(theta)^2);
    A(1,3) =    -dt;
    A(2,1) =     dt * (-q*sin(phi) - r*cos(phi));
    A(2,2) = 1;
    A(2,4) =    -dt;

    xdot = zeros(5,1);

    xdot(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    xdot(2) = q*cos(phi) - r*sin(phi);   
    xdot(3:5) = 0;

    xp = x + xdot*dt;

    Pp = A*P*A' + Q;
    K = Pp*H'/(H*Pp*H' + R);
    dz = [a_phi a_theta]' - H*xp;
    x = xp + K * dz; 
    P = Pp - K*H*Pp;

    EulerSaved(k,:) = [x(1) x(2) psi_filterd];
    disp([x(1)*(180/pi) x(2)*(180/pi) psi_filterd*(180/pi)]);
end

PhiSaved = EulerSaved(:,1) * (180/pi);
ThetaSaved = EulerSaved(:,2) * (180/pi);
PsiSaved = EulerSaved(:,3) * (180/pi);

t = 0:dt:Nsamples*dt-dt;

figure
plot(t, PhiSaved)
ylim([-70 70])
title('Roll (Phi)')

figure
plot(t, ThetaSaved)
ylim([-70 70])
title('Pitch (Theta)')

figure
plot(t, PsiSaved)
ylim([-70 70])
title('Yaw (Psi)')
