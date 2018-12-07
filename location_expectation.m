% this program use the prior information to estimate the errors generated from moving and update the
% lidar scan data to reduce the errors.
function [Lidar_expect_xy,detected_expect_reflector_polar,pose_expect,xy_vel_acc]=location_expectation(scan_freq,detected_reflector_polar,detected_ID,reflector_index,data_len,Lidar_trace,rotation_trace,dist_err_trace,angle_err_trace)


t=1/scan_freq;
x_offset1 = Lidar_trace(end,1)-Lidar_trace(end-1,1);
y_offset1 = Lidar_trace(end,2)-Lidar_trace(end-1,2);
angle_offset1 = rotation_trace(end)-rotation_trace(end-1);
x_offset2 = Lidar_trace(end-1,1)-Lidar_trace(end-2,1);
y_offset2 = Lidar_trace(end-1,2)-Lidar_trace(end-2,2);
Lidar_expect_xy = [Lidar_trace(end,1)+x_offset1 Lidar_trace(end,2)+y_offset1];
angle_offset2 = rotation_trace(end-1)-rotation_trace(end-2);
x_vel=(x_offset1)/t;  % x velocity
y_vel=(y_offset1)/t;  % y velocity
x_acc=(x_offset1-x_offset2)/t^2;  % x acc
y_acc=(y_offset1-y_offset2)/t^2;  % y acc
rot_vel=angle_offset1/t;
rot_acc=(angle_offset1-angle_offset2)/t^2;
angle_err_mean=mean(angle_err_trace(end,:));
dist_err_mean=mean(dist_err_trace(end,:));
%pose_expect = pose_hist+angle_offset1; % assume the angle is not changing during the moving
pose_expect = rotation_trace(end)+angle_offset1;
data = detected_reflector_polar';
[detected_reflector_xy,scan_data] = PolarToRect(data);  
%detected_reflector_xy=detected_reflector_xy';
%size(detected_reflector_xy)
detected_reflector_expect(detected_ID,1) = detected_reflector_xy(detected_ID,1)-x_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len)
detected_reflector_expect(detected_ID,2) = detected_reflector_xy(detected_ID,2)-y_vel*t*((data_len-reflector_index(1,detected_ID)')/data_len)
detected_expect_reflector_polar(detected_ID,1) = angle(detected_reflector_expect(detected_ID,1)+detected_reflector_expect(detected_ID,2)*i)/pi*180;
%detected_expect_reflector_polar(detected_ID,1) = detected_reflector_polar(detected_ID,1)-pose_expect;
detected_expect_reflector_polar(detected_ID,2) = (detected_reflector_expect(detected_ID,1).^2+detected_reflector_expect(detected_ID,2).^2).^0.5;
xy_vel_acc=[x_vel y_vel;x_acc y_acc];
%%%%%%%%%%%%%%%%%%%%%%
% Plot Lidar velocity and acceleration
