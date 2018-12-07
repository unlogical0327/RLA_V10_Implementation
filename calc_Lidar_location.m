%% This program calculate the x and y of Lidar from angle and distance measurement data
function [Lidar_x,Lidar_y,rotation] = calc_Lidar_location(reflect_pool_xy,reflect_pool_ID,matched_detected_polar,matched_detect_ID)
% Calculate xy from angle data
matched_detect_ID;
matched_detected_polar;
for ii=1:length(matched_detect_ID)-1
    alpha(ii)=matched_detected_polar(ii+1,1)-matched_detected_polar(ii,1); % angle between A and B
end
%    x_angle=
%    y_angle=
    
    Xa= reflect_pool_xy(:,1);  % x of reflector from reflector map
    Ya= reflect_pool_xy(:,2);  % y of reflector from reflector map
% Calculate xy from distance data
    Rad = matched_detected_polar(:,2);
    if length(Rad)>3
        calc_round=length(Rad)-3;
    elseif length(Rad)==3
        calc_round=1;
    else
        disp('No enough reflectors found')
    end
    %calc_round = floor(length(Rad)/3);
    for i=1:calc_round
%% -- solve x and y from at least 3 points intersection point
%    x_dist(i) =-(Rad(i)^2*Ya(i+1)-Rad(i+1)^2*Ya(i)-Rad(i)^2*Ya(i+2)+Rad(i+2)^2*Ya(i)+Rad(i+1)^2*Ya(i+2)-Rad(i+2)^2*Ya(i+1))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
%    y_dist(i) =(Rad(i)^2*Xa(i+1)-Rad(i+1)^2*Xa(i)-Rad(i)^2*Xa(i+2)+Rad(i+2)^2*Xa(i)+Rad(i+1)^2*Xa(i+2)-Rad(i+2)^2*Xa(i+1))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
    x_dist(i) =-(Rad(i)^2*Ya(i+1)-Rad(i+1)^2*Ya(i)-Rad(i)^2*Ya(i+2)+Rad(i+2)^2*Ya(i)+Rad(i+1)^2*Ya(i+2)-Rad(i+2)^2*Ya(i+1)-Xa(i)^2*Ya(i+1)+Xa(i+1)^2*Ya(i)+Xa(i)^2*Ya(i+2)-Xa(i+2)^2*Ya(i)-Xa(i+1)^2*Ya(i+2)+Xa(i+2)^2*Ya(i+1)+Ya(i)*Ya(i+1)^2-Ya(i)^2*Ya(i+1)-Ya(i)*Ya(i+2)^2+Ya(i)^2*Ya(i+2)+Ya(i+1)*Ya(i+2)^2-Ya(i+1)^2*Ya(i+2))/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
    y_dist(i) =(Rad(i)^2*Xa(i+1)-Rad(i+1)^2*Xa(i)-Rad(i)^2*Xa(i+2)+Rad(i+2)^2*Xa(i)+Rad(i+1)^2*Xa(i+2)-Rad(i+2)^2*Xa(i+1)+Xa(i)*Xa(i+1)^2-Xa(i)^2*Xa(i+1)-Xa(i)*Xa(i+2)^2+Xa(i)^2*Xa(i+2)+Xa(i+1)*Xa(i+2)^2-Xa(i+1)^2*Xa(i+2)+Xa(i)*Ya(i+1)^2-Xa(i+1)*Ya(i)^2-Xa(i)*Ya(i+2)^2+Xa(i+2)*Ya(i)^2+Xa(i+1)*Ya(i+2)^2-Xa(i+2)*Ya(i+1)^2)/(2*(Xa(i)*Ya(i+1)-Xa(i+1)*Ya(i)-Xa(i)*Ya(i+2)+Xa(i+2)*Ya(i)+Xa(i+1)*Ya(i+2)-Xa(i+2)*Ya(i+1)));
  
    end
    Lidar_x=mean(x_dist);
    Lidar_y=mean(y_dist);
    data=matched_detected_polar';
[matched_detected_xy,scan_data]=PolarToRect(data);  
% reflect_pool_xy;
% matched_detected_xy;
A1=[reflect_pool_xy(1:length(reflect_pool_ID),1) reflect_pool_xy(1:length(reflect_pool_ID),2)];
B1=[matched_detected_xy(1:length(matched_detect_ID),1) matched_detected_xy(1:length(matched_detect_ID),2)];
n_t=length(A1);
[ret_R,ret_T]=rigid_transform_2D(A1, B1);
rotation=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;  % find the rotation angle from Lidar 
% Lidar_xy=[Lidar_x;Lidar_y]';
% Lidar_update_xy=(ret_R^-1*(Lidar_xy'-repmat(ret_T, 1, 1)))';  % get the x y in world coordinate

% Lidar_x1=-1.8190e-12;
% Lidar_y1=-2.8422e-14;
% [ret_R1, ret_T1, Lidar_update_xy]=locate_reflector_xy(reflect_pool_xy,reflect_pool_ID,matched_detected_xy,matched_detect_ID,Lidar_x1,Lidar_y1)