function [output_pt, simul_len, slope_degree] = Stright_Line_Constructor_2dot (start_point,end_point,input_map,step)
% input_map=weight_EC;
% start_point=pt_track_fin(:,end-1);
% end_point=pt_track_fin(:,end);
% step=1;

delta_x=end_point(1)-start_point(1);
delta_y=end_point(2)-start_point(2);
slope=delta_y/delta_x;
slope_degree=atand(slope); 
if delta_y < 0 && delta_x > 0 
    
elseif delta_y < 0 && delta_x < 0 
    slope_degree=slope_degree-180; 
elseif delta_y > 0 && delta_x < 0 
    slope_degree=slope_degree+180; 
else
    
end
    

if delta_x~=0 && abs(delta_x) < abs(delta_y)
    simul_y=[min(start_point(2),end_point(2)):step:max(start_point(2),end_point(2))];
    simul_x_deci=(1/slope)*(simul_y-start_point(2))+start_point(1);
    simul_x=round(simul_x_deci);
    simul_len=length(simul_y);
elseif delta_x~=0 && abs(delta_x) >= abs(delta_y)     
    simul_x=[min(start_point(1),end_point(1)):step:max(start_point(1),end_point(1))];
    simul_y_deci=slope*(simul_x-start_point(1))+start_point(2);
    simul_y=round(simul_y_deci);
    simul_len=length(simul_x);
elseif delta_x == 0
    simul_y=[min(start_point(2),end_point(2)):step:max(start_point(2),end_point(2))];
    simul_x=ones(1,length(simul_y))*start_point(1);
    simul_len=length(simul_x);
else
    error('Slope Error! Straight Line Cannot Be Constructed!'); 
end

simul_z=zeros(1,simul_len);
for ind_i=1:simul_len
    simul_z(ind_i)=input_map(simul_y(ind_i),simul_x(ind_i));
end

output_pt=[simul_x;simul_y;simul_z];