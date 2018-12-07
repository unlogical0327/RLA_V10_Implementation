%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [status,detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,Lidar_Table,Lidar_data)
% Lidar data: reference data
% Lidar Table: new data
detected_ID=0;
iii=0;
detected_reflector=0;
Lidar_Table(:,3)=Lidar_data(2,:);
for ii=2:length(Lidar_data)-1
    if Lidar_data(3,ii)>=amp_thres && Lidar_data(2,ii)>=dist_thres
            % detect all possible peaks
            if Lidar_data(3,ii)>Lidar_data(3,ii-1) && Lidar_data(3,ii)>Lidar_data(3,ii+1)
                iii=iii+1;
                detected_ID_total(iii)=iii;
                detected_reflector_x(detected_ID_total(iii))=Lidar_Table(ii,1);
                detected_reflector_y(detected_ID_total(iii))=Lidar_Table(ii,2);
                detected_reflector_angle(detected_ID_total(iii))=Lidar_data(1,ii);
                detected_reflector_dist(detected_ID_total(iii))=Lidar_data(2,ii);
                raw_reflector_index(iii)=ii;
                disp('Raw detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID_total(iii)));
            end
    end
end
% -- Use LSF to find the optimized reflector location
iii;
if (iii<=1)  % in case no reflector detected
    detected_reflector=0;
    detected_reflector_polar=0;
    disp('No reflector Detected!!!');
else
%%%%--- merge all peaks from the same refletor
   %-- initilze array 1st cell
   l=1;iii=1;
   reflector_index(1)=(raw_reflector_index(1));
   detected_ID(1)=1;
   detected_reflector(1,1) = detected_reflector_x(1);
   detected_reflector(1,2) = detected_reflector_y(1);
   detected_reflector_polar(1,1) = detected_reflector_angle(1);
   detected_reflector_polar(1,2) = detected_reflector_dist(1);
   detected_reflector_angle_g(1) = detected_reflector_angle(1);

   for ii=(l+1):length(detected_reflector_angle)
               angle_delta(ii)=reflector_diameter/Lidar_data(2,ii)/pi*180;
               if (abs(detected_reflector_dist(ii)-detected_reflector_dist(l))<distance_delta) && (abs(detected_reflector_angle(ii)-detected_reflector_angle(l))<angle_delta(ii))  
             % check if the detected point is from the same reflector
                reflector_index(iii)=round((raw_reflector_index(ii)+reflector_index(iii))/2);
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1) = (detected_reflector_x(ii)+detected_reflector(detected_ID(iii),1))/2;
                detected_reflector(detected_ID(iii),2) = (detected_reflector_y(ii)+detected_reflector(detected_ID(iii),2))/2;
                detected_reflector_angle_g(detected_ID(iii)) = (detected_reflector_angle(ii)+detected_reflector_angle_g(detected_ID(iii)))/2;
                detected_reflector_polar(detected_ID(iii),1) = (detected_reflector_angle(ii)+detected_reflector_polar(detected_ID(iii),1))/2;
                detected_reflector_polar(detected_ID(iii),2) = (detected_reflector_dist(ii)+detected_reflector_polar(detected_ID(iii),2))/2;               
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                iii=iii+1;
                reflector_index(iii)=(raw_reflector_index(ii));
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1) = detected_reflector_x(ii);
                detected_reflector(detected_ID(iii),2) = detected_reflector_y(ii);
                detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),1) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),2) = detected_reflector_dist(ii);
                disp('Detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID(iii)));
               end
               l=l+1;
   end
   
   ll=length(detected_reflector_angle_g);
if detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<(angle_delta(2)-360) || detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<angle_delta(2) 
    if (abs(detected_reflector_dist(1)-detected_reflector_dist(end))<distance_delta)
   detected_reflector(1,1) = (detected_reflector(ll,1)+detected_reflector(1,1))/2;
   detected_reflector(1,2) = (detected_reflector(ll,2)+detected_reflector(1,2))/2;
   detected_reflector(ll,:) = [];
   detected_ID(ll) = [];
   detected_reflector_angle_g(ll) = [];
    end
end

end
detected_ID
if (length(detected_ID)<=1 || detected_ID(1)==0)  % in case no reflector detected
    detected_reflector=0;
    detected_reflector_polar=0;
    disp('No reflector Detected!!!');
    reflector_index = 0;
    status = 'No_Ref';
else
    status = 'Ref Found';
end

% -- Use Gaussian fit to find the optimized reflector centra location
N_fit=2;
if ref_gauss_data_fit==1
 for ii=1:length(detected_ID)
    for jj=1:(2*N_fit+1)
        reflector_index(ii)+jj-N_fit-1;
        reflector_data(jj,1)=Lidar_data(1,reflector_index(ii)+jj-N_fit-1);
        reflector_data(jj,2)=Lidar_data(2,reflector_index(ii)+jj-N_fit-1);
        reflector_data(jj,3)=Lidar_data(3,reflector_index(ii)+jj-N_fit-1);
    end
    reflector_data;
    [angle_center_fit,r_center_fit,max_fit_amp] = LSF_ref_center(reflector_data);
    detected_reflector(ii,1) = cos(angle_center_fit/180*pi)*r_center_fit;
    detected_reflector(ii,2) = sin(angle_center_fit/180*pi)*r_center_fit;
    detected_reflector_angle_g(ii) = angle_center_fit;
    detected_reflector_polar(ii,1) = angle_center_fit;
    detected_reflector_polar(ii,2) = r_center_fit;
end
end
detected_reflector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-- Below code tests the algorithm to find and merge conditional detected reflectors 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aa=[   1525
%     1524
%     1523
%     1520
%         1487
%         1484
%         1480
%         1407
%         1314
%         1303
%         12990
%         12380];
%     l=1;
%     k=1;
%     zz(1)=aa(1);
% for i=(l+1):length(aa)
%     if abs(aa(l)-aa(i))<10
%     zz(k)=(aa(l)+aa(i))/2;
%     else 
%         k=k+1;
%         zz(k)=aa(i);
%     end
%     l=l+1;
% end