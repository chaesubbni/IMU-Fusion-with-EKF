function [accel, gyro, mag, soft, hard] = calcurate_sensor_noise(dt)


disp("천천히 회전시켜!!!!!");
pause(3);

calib_points = 500;
mag_cal = zeros(calib_points, 3);
for i = 1: calib_points
    [~,~,m] = imu_read();
    mag_cal(i,:) = m;
    pause(dt);
end

[soft, hard] = magcal(mag_cal);
disp("끝");
pause(5);





num_points = 500;

accel=[];
gyro=[];
mag=[];


disp("센서 노이즈 측정");
pause(5);
%정지값으로 측정.
for i = 1:num_points
    [a,g,m] = imu_read(); 
    accel = [accel;a];
    gyro = [gyro;g];
    mag = [mag;m];
    pause(dt);
end



disp("센서 노이즈 계산 완료");
