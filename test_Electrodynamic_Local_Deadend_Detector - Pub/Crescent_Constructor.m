function [output_pt, z_weight, theta, simul_len] = Crescent_Constructor (center_point,radius,terminal_degree,theta_dev,input_map,weight_map)
% input_map=weight_EC;
% weight_map=weight_CN;
% center_point=vector_orig;
% radius=search_step;

theta0=terminal_degree; 
theta_dev=theta_dev; 
theta=[theta0-theta_dev:1:theta0+theta_dev];
t_len=length(theta);
r=radius;
x=round(r*cosd(theta)+center_point(1));
y=round(r*sind(theta)+center_point(2));
x(x<=0)=1;
y(y<=0)=1;
z=zeros(size(x));
z_weight=zeros(size(x));
for ind_i=1:t_len
    z(ind_i)=input_map(y(ind_i),x(ind_i));
    z_weight(ind_i)=weight_map(y(ind_i),x(ind_i));
end
output_pt=[x;y;z];
[~,simul_len]=size(output_pt);

% figure (1)
% mesh(input_map);
% % imshow(input_map);
% % set(gca,'ydir','normal');
% hold;
% plot3(x,y,z+255); 
% plot3(center_point(1),center_point(2),center_point(3),'ro'); 
% plot3(end_pt(1),end_pt(2),end_pt(3),'r*'); 
% hold;