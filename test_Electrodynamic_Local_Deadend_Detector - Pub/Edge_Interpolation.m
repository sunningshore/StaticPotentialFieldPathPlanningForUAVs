function [x_limit_set,y_limit_set]= Edge_Interpolation (input_x,input_y) 
bound_len=length(input_x);
bound_x_rescale_ini=round(input_x);
bound_y_rescale_ini=round(input_y);
bound_x_rescale=bound_x_rescale_ini;
bound_y_rescale=bound_y_rescale_ini;
x_limit_set=[];
y_limit_set=[];
for ind_i=1:bound_len
    if ind_i==bound_len
        x_temp=[bound_x_rescale(ind_i),bound_x_rescale(1)];
        y_temp=[bound_y_rescale(ind_i),bound_y_rescale(1)];
    else
        x_temp=[bound_x_rescale(ind_i),bound_x_rescale(ind_i+1)];
        y_temp=[bound_y_rescale(ind_i),bound_y_rescale(ind_i+1)];
    end
    slope_temp=(y_temp(2)-y_temp(1))/(x_temp(2)-x_temp(1));
    x_set=[min(x_temp):max(x_temp)];
    y_set=slope_temp*(x_set-x_temp(1))+y_temp(1);
    x_set_len=length(x_set);
    x_limit_set=cat(2,x_limit_set,x_set);
    y_limit_set=cat(2,y_limit_set,y_set);
end