function [track_regul_order, regul_index_order] = test_Trajectory_Regulating (grid_network, pt_track)
%% test Trajectory Regulating %% 
% tic;
% grid_network=grid_network;
% pt_track=pt_track;

[track_dim,track_len]=size(pt_track);
grid_x=grid_network(:,1);
grid_y=grid_network(:,2);
grid_z=grid_network(:,3);
track=[pt_track;[1:track_len]];
track_level=sort(unique(track(3,:)));
track_level_len=length(track_level);
grid_pt_num=zeros(1,track_level_len);
for ind_lv=1:track_level_len 
    lv_temp=track_level(ind_lv); 
    grid_pt_temp=numel(find(grid_z==lv_temp)); 
    grid_pt_num(ind_lv)=grid_pt_temp;
end
track_regul=[];
regul_index=[];
for ind_i=1:track_level_len 
    level_temp=track_level(ind_i); 
    track_temp_ind=find(pt_track(3,:)==level_temp);
%     track_temp_ind=track(4,:);
    track_temp=track(:,track_temp_ind)';
    track_temp_len=length(track_temp_ind);
    track_regul_temp=track_temp;
    regul_index_temp=zeros(track_temp_len,1);
    grid_temp=grid_network(grid_z==level_temp,:);
%     grid_temp=grid_network;
    [grid_temp_len,~]=size(grid_temp);
    for ind_j=1:track_temp_len 
        pt_temp=track_temp(ind_j,[1:3]);
        dist_temp=sqrt(sum((ones(grid_temp_len,1)*pt_temp-grid_temp).^2,2)); 
        [~,dist_min_ind]=min(dist_temp); 
        track_regul_temp(ind_j,[1:3])=grid_temp(dist_min_ind,:);
        regul_index_temp(ind_j)=dist_min_ind;
    end
    track_regul=cat(1,track_regul,track_regul_temp);
    regul_index=cat(1,regul_index,regul_index_temp);
end
track_order=track_regul(:,4);
[~,track_order_ind]=sort(track_order);
track_regul_order=track_regul(track_order_ind,:)'; 
regul_index_order=regul_index(track_order_ind)';
% track_regul_uiq=unique(track_regul_order(1:3,:)','rows')';
%% Plotting %% 
% figure(3)
% plot3(grid_x,grid_y,grid_z,'.');
% hold;
% plot3(track_regul_order(1,:),track_regul_order(2,:),track_regul_order(3,:),'-r.');
% % plot3(grid_x(regul_index),grid_y(regul_index),grid_z(regul_index),'-ro');
% hold;
% grid on;
% toc;