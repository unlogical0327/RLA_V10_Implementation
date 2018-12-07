%% Measurement mode module
% -- this is the measurement mode module used to conduct the continuing
% measurement when Lidar starts to measure the location
function [mea_status,Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,matched_reflect_ID,reflector_index,match_detected_pool,matched_detect_ID,dist_err,angle_err,Reflector_map_wm,wm_reflect_ID,unmatched_detected_reflector,unmatched_detect_ID] = measurement_mode(num_match_pool_orig,num_detect_pool,scan_freq,Reflector_map,Reflector_ID,measurement_data3,scan_data,moving_estimate,ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,Lidar_expect_trace,rotation_trace,dist_err_trace,angle_err_trace,thres_dist_match,thres_dist_large,thres_angle_match)
%% 1. Read the scan data, identify reflectors and define how many scanned reflectors are used from the list(nearest distance or most distingushed).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% identify the variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global matched_ref_ID_hist;
global matched_reflect_vec_ID_hist;
global matched_detect_ID_hist;
global matched_reflect_angle_ID_hist;
global Reflect_dist_vector_hist;
global Reflect_angle_vector_hist;
global xy_vel_acc;
global map_rmse;
global reflector_rmse;
global wm_reflect_ID_hist;
global match_quality;
%
xy_vel_acc = [0 0;0 0];
Lidar_expect_xy_old=Lidar_expect_trace(end,:);
%%%%%%%%%%%%%%%%%%%%%
% measurement data: new data
% scan data: reference data
measurement_data(:,1)=measurement_data3(:,1);
measurement_data(:,2)=measurement_data3(:,2);
[data_len,data_w]=size(measurement_data);
Lidar_data=scan_data;
[calibration_data,scan_data]=PolarToRect(Lidar_data);
%[ref_status1,detected_ID1,detected_reflector1,detected_reflector_polar1,reflector_index1]=identify_reflector(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,calibration_data,Lidar_data);
%detected_ID1
[ref_status,detected_ID,detected_reflector_polar,reflector_index]=identify_reflector_polar(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_data)
% Use moving model to estimate the location of each of detected reflector
% with estimated velocity 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For the case less than 3 reflectors detected.
if length(detected_ID)<3 
    disp('detected reflectors smaller than 2....Bad data and wait for another scan data, use previous data.....')
    %-- pass the null value to output
    reflector_rmse=99.99;
    Lidar_trace=[0 0];
    rotation_trace=0;
    Lidar_expect_trace=[0 0];
    Lidar_update_Table=0;
    detected_ID2=0;
    detected_reflector2=0;
    match_reflect_pool=0;
    matched_reflect_ID=0;   
    matched_detect_ID=0;
    match_detected_pool=0;
    Reflector_map_wm=0;
    wm_reflect_ID=0;
    unmatched_detected_reflector=0;
    unmatched_detect_ID=0;
    rotation_trace=[0 0];
    dist_err=0;
    angle_err=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    
[ll,ww]=size(Lidar_trace);
if ll>=3  && moving_estimate ==1
  %[detected_expect_reflector,xy_vel_acc]=update_scan_data_estimation(scan_freq,detected_reflector,detected_ID,reflector_index,data_len,pose_hist)
  [Lidar_expect_xy,detected_expect_reflector_polar,pose_expect,xy_vel_acc]=location_expectation(scan_freq,detected_reflector_polar,detected_ID,reflector_index,data_len,Lidar_trace,rotation_trace,dist_err_trace,angle_err_trace)
%    detected_reflector;
%    detected_expect_reflector;
    detected_reflector_polar = detected_expect_reflector_polar;
else
    Lidar_expect_xy=Lidar_trace(end,:);   % Need to add expectation xy
    pose_expect=rotation_trace(end,:);   % Need to add expectation xy
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_match_pool=num_match_pool_orig;
detected_reflector_polar_orig=detected_reflector_polar;
detected_ID_orig=detected_ID;
Lidar_current_xy=[0 0];
theta_rot=0;  
%% 2. Match the N x scanned reflectors with match reflector table and find the location of lidar.
% Detect if pool size is set to less than 3
if num_match_pool<=2
    disp('detected reflector is defined a illegal reflector number to RLA. automatically set to 3 to start from....')
    num_match_pool=3;
end
thres_near = 400;   % near threshold in mm
thres_far = 30000;   % far threshold in mm
bad_detected_ID=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pick up reflectors from scan data and match
[update_detected_reflector_polar,update_detected_ID,detected_pool_polar,detected_pool_ID,detected_dist_max,detected_status]=generate_detected_pool(num_match_pool,detected_reflector_polar,detected_ID,reflector_index,bad_detected_ID,thres_near,thres_far)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -- create match ref pool referring to detected pool
[reflect_pool_xy,reflect_pool_dist,reflect_pool_angle,reflect_pool_ID,bad_detected_ID,bad_detected_reflector_polar,dist_err,angle_err,pool_status,queue_status,match_status]=generate_reflector_pool(num_match_pool,Reflector_map,Reflector_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID,detected_dist_max,dist_err_trace,angle_err_trace,thres_near,thres_far)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -- below can remove the bad point from detected pool and run test
a=1;
while a==1;
    if match_status==1 && queue_status==0 && detected_status==0
        % -- generate detect pool with data without bad points
        [update_detected_reflector_polar,update_detected_ID,detected_pool_polar,detected_pool_ID,detected_dist_max,detected_status]=generate_detected_pool(num_match_pool,update_detected_reflector_polar,update_detected_ID,reflector_index,bad_detected_ID,thres_near,thres_far)
        [reflect_pool_xy,reflect_pool_dist,reflect_pool_angle,reflect_pool_ID,bad_detected_ID,bad_detected_reflector_polar,dist_err,angle_err,pool_status,queue_status,match_status]=generate_reflector_pool(num_match_pool,Reflector_map,Reflector_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID,detected_dist_max,dist_err_trace,angle_err_trace,thres_near,thres_far)
%     elseif match_status==1 && pool_status==1
%         disp('Cant find enough matched ref reflectors and update detected pool with another data point.......!!!!')
%         % -- generate detect pool with data without bad points
%         [detected_pool_polar,detected_pool_ID,detected_dist_max,detected_status]=generate_detected_pool(num_match_pool,detected_reflector_polar,detected_ID,reflector_index,bad_detected_ID,thres_near,thres_far)
%         [reflect_pool_xy,reflect_pool_dist,reflect_pool_angle,reflect_pool_ID,bad_detected_ID,bad_detected_reflector_polar,dist_err,angle_err,pool_status,queue_status,match_status]=generate_reflector_pool(num_match_pool,Reflector_map,Reflector_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID,detected_dist_max,dist_err_trace,angle_err_trace,thres_near,thres_far) 
    elseif queue_status==1 && match_status==1 && detected_status==0
        disp('Ref pool reach the end of Q and NO match found!!!')
        break
    elseif match_status==0 && detected_status==0  % find matched reflectors and detected reflectors are enough
        disp('Find matched ref pool with detected pool!!!')
        break
    elseif detected_status==1   % No enough detetced reflectors found 
        disp('No enough detected reflectors in detected pool and test failed!!!')
        break       
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
var=0.5;
match_quality_old = match_quality;
if match_status==1  % If NO matched pool, then use previous saved reflectors to continue the test
         reflect_pool_ID = matched_ref_ID_hist';
         wm_reflect_ID = wm_reflect_ID_hist;
         ref_ID=matched_ref_ID_hist;
         [idx,idy]=find(Reflector_ID(:)==ref_ID);
         reflect_pool_xy = Reflector_map(idx,:);
         Lidar_update_xy_wm = Lidar_expect_xy;
         rotation=pose_expect;
         bad_detected_ID=0;
         disp('Matching failed! Use previous expected location to continue the test.....')
         match_result=1;  
% -- Check if use previous reflector pool can match
elseif match_status==0
         disp('Matching succeed! Continue to calculate Lidar location in world MAP!!')
         [match_quality,match_result]=match_reflector_pool(reflect_pool_xy,reflect_pool_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check match quality
         if match_quality>1000    %abs(match_quality-match_quality_old)>abs(match_quality_old*var) && ll>=3    %match_quality>1000 %&& match_status==0    %
             reflect_pool_ID = matched_ref_ID_hist';
             wm_reflect_ID = wm_reflect_ID_hist;
             ref_ID=matched_ref_ID_hist;
             [idx,idy]=find(Reflector_ID(:)==ref_ID)
             reflect_pool_xy = Reflector_map(idx,:);  
             Lidar_expect_xy=Lidar_expect_xy_old;
             [match_quality,match_result]=match_reflector_pool(reflect_pool_xy,reflect_pool_ID,Lidar_expect_xy,pose_expect,detected_pool_polar,detected_pool_ID);
         end
         if match_result==0
            disp('detected reflectors pool matched with ref reflector pool.....')
            disp(sprintf('Match Quality: %i', match_quality));
         else
            disp('No matched reflectors found....')
         end
         [Lidar_x_wm,Lidar_y_wm,rotation] = calc_Lidar_location(reflect_pool_xy,reflect_pool_ID,detected_pool_polar,detected_pool_ID);
          Lidar_update_xy_wm = [Lidar_x_wm Lidar_y_wm];
end

%% plot map new Lidar scan
%% plot map with random displacement
%plot_reflector(detected_reflector1,detected_ID1,color)
%% 2.c calculate rotation and transition
match_reflect_pool=reflect_pool_xy;
data=detected_pool_polar';
[match_detected_pool,detected_pool_polar1]=PolarToRect(data);  
matched_reflect_ID=reflect_pool_ID;
matched_detect_ID=detected_pool_ID;
Lidar_x=0;  % This is Lidar x in Lidar's view and used to convert to world map 
Lidar_y=0;  % This is Lidar y in Lidar's view and used to convert to world map 

if match_result == 0
    if length(matched_reflect_ID)==length(matched_detect_ID)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %- Sort the sequence to make rotation order right
        len=length(matched_reflect_ID);
        sort_Q=sort(matched_reflect_ID);
        idx=find(sort_Q==matched_reflect_ID(1));
        for ii=1:len
            if idx+ii-1>len
             matched_reflect_ID(ii) = sort_Q(idx+ii-1-len);
            else
             matched_reflect_ID(ii) = sort_Q(idx+ii-1);
            end
        end
        reflector_rmse_old = reflector_rmse;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Convert Lidar scan data to world map data
        %data=detected_reflector_polar_orig';
        %[detected_reflector_xy1,detected_reflector_polar1]=PolarToRect(data); 
        [Lidar_update_Table,Lidar_update_xy,theta_rot,reflector_rmse,map_rmse]=convert_reflector_pool_world_map(measurement_data,scan_data,match_reflect_pool,matched_reflect_ID,match_detected_pool,matched_detect_ID,Lidar_x,Lidar_y);        
        delta_xy=((Lidar_trace(end,1)-Lidar_update_xy(1,1))^2+(Lidar_trace(end,2)-Lidar_update_xy(1,2))^2)^0.5;
        rmse_thres_var = 0.15;    % Define the rmse change to trigger to use old reflector mapping
        xy_jump=400;   % threshold to tell if there is an error jump, Not larger than speed
        delta_xy

    if match_quality>1000 || delta_xy>xy_jump    %abs(reflector_rmse-reflector_rmse_old) > abs(reflector_rmse_old*rmse_thres_var) || delta_xy>xy_jump   % exceptional condition to error jump
        matched_reflect_ID = matched_ref_ID_hist';
         matched_detect_ID = matched_detect_ID_hist';
         wm_reflect_ID = wm_reflect_ID_hist;
         [Lidar_update_Table,Lidar_update_xy,theta_rot,reflector_rmse,map_rmse]=convert_reflector_pool_world_map(measurement_data,scan_data,match_reflect_pool,matched_reflect_ID,match_detected_pool,matched_detect_ID,Lidar_x,Lidar_y);        
         12345
    else
         matched_ref_ID_hist = matched_reflect_ID';
         matched_detect_ID_hist = matched_detect_ID';
         54321
    end    
        %% Plot the reflectors in the world map
        %% Plot update map in the world map
        unmatched_detected_reflector=0;
        unmatched_detect_ID=0;
        wm_reflect_ID=matched_reflect_ID';
        [idx,idy]=find(Reflector_ID(:)==wm_reflect_ID);
        Reflector_map_wm=Reflector_map(idx,:);
        %% --Update Lidar trace
%        Lidar_trace=[Lidar_trace;Lidar_update_xy];
        Lidar_trace=[Lidar_trace;Lidar_update_xy_wm];        
        Lidar_expect_trace=[Lidar_expect_trace;Lidar_expect_xy];
        rotation_trace=[rotation_trace;theta_rot];

    end
elseif match_result == 1
    disp('Bad data and wait for another scan data, use previous data.....');
        reflector_rmse=99.99;
        Lidar_update_Table=0;
        match_reflect_pool=0;
        matched_reflect_ID=0;   
        matched_detect_ID=0;
        Reflector_map_wm=0;
        wm_reflect_ID=0;  
         unmatched_detected_reflector=0;
         unmatched_detect_ID=0;

        %% --Update Lidar trace
%        Lidar_trace=[Lidar_trace;Lidar_update_xy];
        Lidar_trace=[Lidar_trace;Lidar_update_xy_wm];        
        Lidar_expect_trace=[Lidar_expect_trace;Lidar_expect_xy];
        rotation_trace=[rotation_trace;pose_expect];
end

end

if reflector_rmse<1
    mea_status=0;
elseif reflector_rmse>1 && reflector_rmse<10
    mea_status=1;
elseif reflector_rmse>10 && reflector_rmse<99.99
    mea_status=2;
elseif reflector_rmse==99.99
    mea_status=3;
else
    mea_status=4;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end  % Simulation loop end up here!!!


