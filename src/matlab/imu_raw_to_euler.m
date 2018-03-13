function [ euler_ang, gyro_dps ] = imu_raw_to_euler( quaternion_raw, gyro_raw)
%Converts raw data (range -32767 to 32767) to proper units

% Gyroscope has a max value of 250 degrees per second
gyro_dps = gyro_raw * (250/32767);
gyro_dps = [gyro_dps(3) -gyro_dps(1) gyro_dps(2)];

% Quarternion should be from 0 to 1
quarternion = quaternion_raw/32767;

qw = quarternion(1);
qx = quarternion(2);
qy = quarternion(3);
qz = quarternion(4);

euler_ang = [ atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ), ...
            asin( -2*(qx.*qz-qw.*qy) ), ...
            atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 )];;

euler_ang = radtodeg(euler_ang);
end

