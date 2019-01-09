function [output_pt, simul_len] = Stright_Line_Constructor_dot_slope (start_point,slope_degree,len,input_map)
% input_map=weight_EC;
% start_point=pt_track_fin(:,end-1);
% end_point=pt_track_fin(:,end);
% delta_x=end_point(1)-start_point(1);
% delta_y=end_point(2)-start_point(2);
% slope=delta_y/delta_x;
% slope_degree=atand(slope);
slope=tand(slope_degree);
[row,col]=size(input_map);
% if perpend_flag == 1 %% perpendicular 
%     if slope_degree > 0 
%         slope_degree=slope_degree-45; 
%     else
%         slope_degree=slope_degree+45; 
%     end
% end

if slope_degree <= 45 && slope_degree >= -45    
%     simul_x=[start_point(1)-round(len/2):1:start_point(1)+round(len/2)];
    simul_x=[start_point(1):1:start_point(1)+round(len)];
    simul_y_deci=slope*(simul_x-start_point(1))+start_point(2);
    simul_y=round(simul_y_deci);
    simul_len=length(simul_x);
elseif (slope_degree >= 135 && slope_degree <= 180) || (slope_degree <= -135 && slope_degree >= -180)  
    simul_x=[start_point(1):-1:start_point(1)-round(len)];
    simul_y_deci=slope*(simul_x-start_point(1))+start_point(2);
    simul_y=round(simul_y_deci);
    simul_len=length(simul_x);
elseif (slope_degree > 45 && slope_degree < 90) || (slope_degree <= 135 && slope_degree > 90)
    simul_y=[start_point(2):1:start_point(2)+round(len)];
    simul_x_deci=(1/slope)*(simul_y-start_point(2))+start_point(1);
    simul_x=round(simul_x_deci);
    simul_len=length(simul_y);
elseif (slope_degree < -45 && slope_degree > -90) || (slope_degree >= -135 && slope_degree < -90)  
    simul_y=[start_point(2):-1:start_point(2)-round(len)];
    simul_x_deci=(1/slope)*(simul_y-start_point(2))+start_point(1);
    simul_x=round(simul_x_deci);
    simul_len=length(simul_y);
elseif slope_degree == 90  
    simul_y=[start_point(2):1:start_point(2)+round(len)];
    simul_x=ones(1,length(simul_y))*start_point(1);
    simul_len=length(simul_x);
elseif slope_degree == -90 
    simul_y=[start_point(2):-1:start_point(2)-round(len)];
    simul_x=ones(1,length(simul_y))*start_point(1);
    simul_len=length(simul_x);
else 
    error('No Such Slope!');
end

simul_z=zeros(1,simul_len);
for ind_i=1:simul_len
    if simul_y(ind_i) < 1 || simul_x(ind_i) < 1 || simul_y(ind_i)>row || simul_x(ind_i)>col 
        simul_z(ind_i)=0;
    else
        simul_z(ind_i)=input_map(simul_y(ind_i),simul_x(ind_i));
    end
end

output_pt=[simul_x;simul_y;simul_z];