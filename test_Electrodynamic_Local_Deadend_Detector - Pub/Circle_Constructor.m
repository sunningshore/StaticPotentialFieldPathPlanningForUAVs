function [output_pt, bound_z, simul_len] = Circle_Constructor (center_point,radius,theta,input_map,bound_map)
% input_map=weight_EC;
% center_point=end_pt;
% % radius=280;

[map_row,map_col]=size(input_map);
t_len=length(theta);
r=radius;
x=round(r*cosd(theta)+center_point(1));
y=round(r*sind(theta)+center_point(2));
x(x>map_col)=map_col;
y(y>map_row)=map_row;
x(x<1)=1;
y(y<1)=1;
z=zeros(size(x));
bound_z=zeros(size(x));
for ind_i=1:t_len
    z(ind_i)=input_map(y(ind_i),x(ind_i));
    bound_z(ind_i)=bound_map(y(ind_i),x(ind_i));
end
output_pt=[x;y;z];
[~,simul_len]=size(output_pt);

% figure (1)
% % mesh(input_map);
% imshow(input_map);
% set(gca,'ydir','normal');
% hold;
% plot3(x,y,z);
% hold;