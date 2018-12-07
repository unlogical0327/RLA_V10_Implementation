%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [status,detected_ID,detected_reflector_polar,reflector_index]=identify_reflector_polar(ref_gauss_data_fit,amp_thres,dist_thres,reflector_diameter,distance_delta,Lidar_data)
% Lidar data: reference data
detected_ID=0;
iii=0;
detected_reflector_polar=0;
%Lidar_Table(:,3)=Lidar_data(2,:);

for ii=2:length(Lidar_data)-1
    if Lidar_data(3,ii)>=amp_thres && Lidar_data(2,ii)>=dist_thres
            % detect all possible peaks
            if Lidar_data(3,ii)>Lidar_data(3,ii-1) && Lidar_data(3,ii)>Lidar_data(3,ii+1)
                iii=iii+1;
                detected_ID_total(iii)=iii;
                detected_reflector_angle(detected_ID_total(iii))=Lidar_data(1,ii);
                detected_reflector_dist(detected_ID_total(iii))=Lidar_data(2,ii);
                raw_reflector_index(iii)=ii;
                disp('Raw detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID_total(iii)));
            end
    end
end
% detected_reflector_angle
% detected_reflector_dist
% raw_reflector_index
% detected_ID_total
% -- Use LSF to find the optimized reflector location
if (iii<=1)  % in case no reflector detected
    detected_ID=0;
    detected_reflector_polar=0;
    disp('No reflector Detected!!!');
else
%%%%--- merge all peaks from the same refletor
   %-- initilze array 1st cell
   l=1;iii=1;
   reflector_index(1)=(raw_reflector_index(1));
   detected_ID(1)=1;
   detected_reflector_polar(1,1) = detected_reflector_angle(1);
   detected_reflector_polar(1,2) = detected_reflector_dist(1);
   detected_reflector_angle_g(1) = detected_reflector_angle(1);

   for ii=(l+1):length(detected_reflector_angle)
        %for ii=1:length(detected_reflector_angle)
               %angle_delta(ii)=reflector_diameter/Lidar_data(2,ii)/pi*180;
               angle_delta(ii)=reflector_diameter/detected_reflector_dist(ii)/pi*180;
               if (abs(detected_reflector_dist(ii)-detected_reflector_dist(l))<distance_delta) && (abs(detected_reflector_angle(ii)-detected_reflector_angle(l))<angle_delta(ii))  
             % check if the detected point is from the same reflector
                reflector_index(iii)=round((raw_reflector_index(ii)+reflector_index(iii))/2);
                detected_ID(iii)=iii;
                detected_reflector_angle_g(detected_ID(iii)) = (detected_reflector_angle(ii)+detected_reflector_angle_g(detected_ID(iii)))/2;
                detected_reflector_polar(detected_ID(iii),1) = (detected_reflector_angle(ii)+detected_reflector_polar(detected_ID(iii),1))/2;
                detected_reflector_polar(detected_ID(iii),2) = (detected_reflector_dist(ii)+detected_reflector_polar(detected_ID(iii),2))/2;               
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                iii=iii+1;
                reflector_index(iii)=(raw_reflector_index(ii));
                detected_ID(iii)=iii;
                detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),1) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),2) = detected_reflector_dist(ii);
                disp('Detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID(iii)));
               end
               l=l+1;
   end

   ll=length(detected_reflector_angle_g);  % detect if last reflector at 180 is the same with the first one at -180

%  detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<(angle_delta(2)-360) ||
   if  detected_reflector_angle_g(1)+detected_reflector_angle_g(ll)<angle_delta(2) 
        if (abs(detected_reflector_dist(1)-detected_reflector_dist(end))<distance_delta)
            detected_reflector_polar(1,1) = (detected_reflector_polar(ll,1)+detected_reflector_polar(1,1))/2;
            detected_reflector_polar(1,2) = (detected_reflector_polar(ll,2)+detected_reflector_polar(1,2))/2;
            detected_reflector_polar(ll,:) = [];
            detected_ID(ll) = [];
            detected_reflector_angle_g(ll) = [];
        end
    end

end
detected_ID

if (length(detected_ID)<=1 || detected_ID(1)==0)  % in case no reflector detected
    detected_reflector_polar=0;
    disp('No reflector Detected!!!');
    reflector_index = 0;
    status = 'No_Ref';
else
    status = 'Ref Found';
end

% -- Use Gaussian fit to find the optimized reflector centra location
N_fit=2;
st='Ref Found';
tf=strcmp(status,st);
if ref_gauss_data_fit==1 && tf==1
 for ii=1:length(detected_ID)
    for jj=1:(2*N_fit+1)
        index=reflector_index(ii)+jj-N_fit-1;
        if index<1  % wrap around data if reflectors at the beginning and end of Lidar start point 
            index=length(Lidar_data)+index;
        elseif index>length(Lidar_data)
            index=index-length(Lidar_data);
        end
        reflector_data(jj,1)=Lidar_data(1,index);
        reflector_data(jj,2)=Lidar_data(2,index);
        reflector_data(jj,3)=Lidar_data(3,index);
    end
    reflector_data;
    [angle_center_fit,r_center_fit,max_fit_amp] = LSF_ref_center(reflector_data);
    detected_reflector_angle_g(ii) = angle_center_fit;
    detected_reflector_polar(ii,1) = angle_center_fit;
    detected_reflector_polar(ii,2) = r_center_fit;
    end
end
