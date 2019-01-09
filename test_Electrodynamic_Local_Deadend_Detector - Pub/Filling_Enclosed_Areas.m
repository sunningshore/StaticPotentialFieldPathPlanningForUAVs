function output = Filling_Enclosed_Areas (input,input_x,input_y,filling_color)
% filling_color=0;    % White inside
% input=urban_map;
% input_x=max_cont_x;
% input_y=max_cont_y;

bound_len=length(input_x);
bound_x_rescale_ini=round(input_x);
bound_y_rescale_ini=round(input_y);
% y_turn_flag=zeros(1,bound_len);
% x_turn_flag=zeros(1,bound_len);
bin_map=input;
% for ind_h=1:bound_len
%     if ind_h==1 
%         y_left=bound_y_rescale_ini(end);
%         y_right=bound_y_rescale_ini(ind_h+1);
%         x_left=bound_x_rescale_ini(end);
%         x_right=bound_x_rescale_ini(ind_h+1);
%     elseif ind_h==bound_len
%         y_left=bound_y_rescale_ini(ind_h-1);
%         y_right=bound_y_rescale_ini(1);
%         x_left=bound_x_rescale_ini(ind_h-1);
%         x_right=bound_x_rescale_ini(1);
%     else
%         y_left=bound_y_rescale_ini(ind_h-1);
%         y_right=bound_y_rescale_ini(ind_h+1);
%         x_left=bound_x_rescale_ini(ind_h-1);
%         x_right=bound_x_rescale_ini(ind_h+1);
%     end
%     if y_left>bound_y_rescale_ini(ind_h) && y_right>bound_y_rescale_ini(ind_h) 
%         y_turn_flag(ind_h)=1;    % Down Turning Point 
%     elseif y_left<bound_y_rescale_ini(ind_h) && y_right<bound_y_rescale_ini(ind_h) 
%         y_turn_flag(ind_h)=2;    % Up Turning Point 
%     end
%     if x_left>bound_x_rescale_ini(ind_h) && x_right>bound_x_rescale_ini(ind_h)
%         x_turn_flag(ind_h)=1;    % Left Turning Point 
%     elseif x_left<bound_x_rescale_ini(ind_h) && x_right<bound_x_rescale_ini(ind_h) 
%         x_turn_flag(ind_h)=2;    % Right Turning Point 
%     end
% end
% up_turn_ind=find(y_turn_flag==2);
% down_turn_ind=find(y_turn_flag==1);
% right_turn_ind=find(x_turn_flag==2);
% left_turn_ind=find(x_turn_flag==1);
% bound_x_rescale=[bound_x_rescale_ini(left_turn_ind:end),bound_x_rescale_ini(1:left_turn_ind-1)];
% bound_y_rescale=[bound_y_rescale_ini(left_turn_ind:end),bound_y_rescale_ini(1:left_turn_ind-1)];
% % slope_logic_boun=(bound_y_rescale(up_turn_ind)-bound_y_rescale(down_turn_ind))/(bound_x_rescale(up_turn_ind)-bound_x_rescale(down_turn_ind)); 
% x_set_logic_boun=[bound_x_rescale(down_turn_ind),bound_x_rescale(up_turn_ind)];
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
%     if ~isnan(slope_temp)
        x_set=[min(x_temp):max(x_temp)];
        y_set=slope_temp*(x_set-x_temp(1))+y_temp(1);
        x_set_len=length(x_set);
        x_limit_set=cat(2,x_limit_set,x_set);
        y_limit_set=cat(2,y_limit_set,y_set);
%     end
%     if slope_temp < 0 
%         slope_flag=1;
%     elseif slope_temp > 0
%         slope_flag=0;
%     else
%         continue; 
%     end
%     if max(x_set) > max(x_set_logic_boun) 
%     elseif min(x_set) < min(x_set_logic_boun) 
%         slope_flag=~slope_flag;
%     end
%     for ind_j=1:x_set_len
%         x_input=x_set(ind_j);
%         y_input=y_set(ind_j);
%         y_output=[1:y_input];
%         bin_map(y_output,x_input)=filling_color*slope_flag;
%     end
end
% limit_len=length(x_limit_set);
for val_x=min(bound_x_rescale):max(bound_x_rescale)
    ind_limit_y=find(x_limit_set==val_x); 
    val_y=y_limit_set(ind_limit_y); 
    up_limit_y=round(max(val_y));
    dn_limit_y=round(min(val_y));
%     if ~isnan(slope_temp)
        bin_map(dn_limit_y:up_limit_y,val_x)=filling_color;
%     end
end
output=bin_map;
% figure (1)
% imshow(output);
% set(gca,'Ydir','normal');
end