function [dist_register, cn_weight_register, theta, t_len] = Circle_Angle_Weight (center_point,terminal_degree,theta_dev,input_map,nearset_contour,connect_weight)
% input_map=weight_EC;
% center_point=vector_orig_vrt;
% nearset_contour;
% connect_weight=weight_CN;
% 
% figure (3)
% imshow(map_img_rgb);
% set(gca,'Ydir','normal');
% % mesh(weight_EC);
% % mesh(connect_weight);
% hold;
% plot(center_point(1),center_point(2),'ro');
% plot(nearset_contour(1,:),nearset_contour(2,:),'r.');

theta_dev=theta_dev; 
[row,col]=size(input_map);
R=sqrt(row^2+col^2);
theta0=terminal_degree;
theta=[theta0-theta_dev:1:theta0+theta_dev];
t_len=length(theta);
dist_register=zeros(1,t_len);
cn_weight_register=zeros(1,t_len);
for ind_t=1:t_len 
    input_slope=tand(theta(ind_t));
    input_degree=theta(ind_t);
    while input_degree > 180 || input_degree < -180
        if input_degree > 180
            input_degree=input_degree-360; 
        else
            input_degree=360+input_degree;
        end
    end
%     input_degree
    [simul_val, simul_len] = Stright_Line_Constructor_dot_slope (center_point,input_degree,R,input_map);
    simul_x=simul_val(1,:);
    simul_y=simul_val(2,:);
    simul_z=simul_val(3,:);
%     [~,~,intersect_ind]=intersect([simul_x;simul_y]',nearset_contour(1:2,:)','rows');
    [~,intersect_ind0,intersect_ind]=intersect([[simul_x;simul_y],[simul_x+1;simul_y],[simul_x-1;simul_y]]',nearset_contour(1:2,:)','rows');
    dist_mat=sqrt(sum((center_point(1:2)*ones(1,length(intersect_ind))-[nearset_contour(1,intersect_ind);nearset_contour(2,intersect_ind)]).^2,1));
    [min_dist_val,min_dist_ind]=min(dist_mat);
    intersect_ind=intersect_ind(min_dist_ind);
    if ~isempty(min_dist_val)
        dist_register(ind_t)=min_dist_val;
    end
    
    intersect_ind0=intersect_ind0(min_dist_ind);
    while intersect_ind0 > simul_len 
        intersect_ind0=intersect_ind0-simul_len;
    end
    simul_len_cn=intersect_ind0;
    cn_weight_z=zeros(1,simul_len_cn);
    for ind_n=1:simul_len_cn 
        cn_weight_z(ind_n)=connect_weight(simul_y(ind_n),simul_x(ind_n));
    end
    cn_weight_register(ind_t)=sum(cn_weight_z);
    
%     plot(nearset_contour(1,intersect_ind),nearset_contour(2,intersect_ind),'rs');
% %     plot([simul_x(1),nearset_contour(1,intersect_ind)],[simul_y(1),nearset_contour(2,intersect_ind)]);
%     plot([simul_x(1),simul_x(intersect_ind0)],[simul_y(1),simul_y(intersect_ind0)]);
end
% hold;
% figure (4);
% % plot(dist_register);
% % plot(cn_weight_register);
% mesh(connect_weight);