%% test Intersection Adjusting %% 
clear;
clc;
tic;
load('weight_EC_cube.mat','weight_EC_cube');
load('map_img_rgb_cube.mat','map_img_rgb_cube');
load('traject_free.mat','traject_free');
load('traject_regul.mat','traject_regul');
load('grid_network.mat','grid_network');
load('air_level.mat','air_level');
load('takeoff_landing_point.mat','takeoff_landing_point');
eval(['load(''cubic_urban_map_',num2str(min(air_level)),'.mat'',''cubic_urban_map'');']);
UAV_vert_guard_dist=5;    % Verticle guard distance between UAVs; 
layer_gap=min(diff(air_level));
sub_space_height=10*layer_gap;    % The height of sub-space, Space_Multiplex_Rate = sub_space_height/layer_gap; 
air_level_len=length(air_level);
[traject_num,~]=size(traject_regul); 
[grid_num,~]=size(grid_network);
od_ind_set=[traject_regul{:,1}]';
traject_level_tube=cell(air_level_len,1);
grid_size=zeros(air_level_len,1);
takeoff_landing_pt_index=takeoff_landing_point(4,:);
traject_len=zeros(traject_num,1);
% traject_regul_adj=traject_regul;
traject_free_adj=traject_free;
%% Grid Size on Each Level %% 
for ind_a=1:air_level_len 
    a_lv=air_level(ind_a);
    grid_size(ind_a)=numel(find(grid_network(:,3)==a_lv));
end
%% Trajectories on Each Level %% 
for ind_i=1:traject_num 
    [od_pt_temp,traject_temp,traject_index_temp]=traject_regul{ind_i,:}; 
%     traject_len(ind_i)=length(traject_index_temp);
    level_set_temp=traject_temp(3,:);
    level_uni_temp=unique(level_set_temp);
    level_len_temp=length(level_uni_temp);
    for ind_j=1:level_len_temp 
        level_temp=level_uni_temp(ind_j);
        level_ind_temp=find(air_level==level_temp);
        level_index=traject_index_temp(level_set_temp==level_temp); 
        if ~isempty(level_index) 
            traject_level_tube{level_ind_temp,:}=cat(1,traject_level_tube{level_ind_temp,:},{ind_i,level_index});
        end
    end
end
%% Intersection Points on Each Level %% 
intersect_level_tube=cell(air_level_len,1);   % Disply waypoints with at least 1 link on each layer, [index of node, index of links through the node]; 
intersect_level_tube_pos=cell(air_level_len,1);    % Display intersection points with more than 1 links on each layer, [index of node, index of links intersecting at the node]; 
level_adjst=zeros(traject_num ,air_level_len);    % Sub-level adjust vector, [1st level, 2nd level, 3rd level];
for ind_l=1:air_level_len 
    grid_size_temp=grid_size(ind_l);
    node_level=[1:grid_size_temp]'; 
    intersect_level_tube{ind_l}=num2cell(node_level);
    intersect_level_tube{ind_l}=cat(2,intersect_level_tube{ind_l},cell(grid_size_temp,1));
    traject_level=traject_level_tube{ind_l,:}; 
    [traject_size,~]=size(traject_level);
    for ind_t=1:traject_size 
        [link_ind,node_ind]=traject_level{ind_t,:}; 
        [intersect_temp,intersect_ind_temp1,intersect_ind_temp2]=intersect(node_level,node_ind);
        intersect_len=length(intersect_ind_temp1);
        for ind_s=1:intersect_len
            intersect_ind=intersect_ind_temp1(ind_s);
            intersect_level_tube{ind_l}{intersect_ind,2}=cat(2,intersect_level_tube{ind_l}{intersect_ind,2},link_ind);
        end
    end
    intersect_num=cell2mat(cellfun(@length,intersect_level_tube{ind_l}(:,2),'UniformOutput',false));
    intersect_level_pos = cellfun(@(x) x(intersect_num > 1,:), intersect_level_tube(ind_l),'UniformOutput',false); 
    intersect_level_tube_pos(ind_l,:)=intersect_level_pos;
    %% Link Clustering %% 
    intersect_pt=intersect_level_pos{1};
    [intersect_size,~]=size(intersect_pt);
    cluster_map=zeros(intersect_size,traject_size);
    for ind_c=1:intersect_size 
        if ind_l==1 && ismember(intersect_pt{ind_c,1},takeoff_landing_pt_index) 
            continue;
        end
        link_temp=intersect_pt{ind_c,2};
        cluster_map(ind_c,[link_temp])=1;
    end
    cluster_ind=sum(cluster_map,1);
    cluster_ind_pos=find(cluster_ind>0);
    cluster_ind_pos_diff=diff(cluster_ind_pos);
    cluster_ind_break_ind=find(cluster_ind_pos_diff>1);
    cluster_ind_break_len=length(cluster_ind_break_ind); 
    cluster_count=cluster_ind_break_len+1; 
    start_ind=[1,cluster_ind_break_ind+1];
    end_ind=[cluster_ind_break_ind,length(cluster_ind_pos)];
    clsuter_len=zeros(cluster_count,1);
    %% Generating Sub-space %% 
    for ind_cl=1:cluster_count 
        cluster_temp=cluster_ind_pos(start_ind(ind_cl):end_ind(ind_cl));
        eval(['cluster_',num2str(ind_cl),'=cluster_temp;']);
        cluster_size=length(cluster_temp);
        clsuter_len(ind_cl)=cluster_size;
        traject_vert_dist=sub_space_height/(cluster_size+1); 
        if traject_vert_dist < UAV_vert_guard_dist 
            eval(['disp(''Oops! Link ',num2str(cluster_temp),' cannot co-exist on air layer ',num2str(ind_l),'. Please Re-plan!'');']);
        else 
            adjst_traject_len=traject_len(cluster_temp);    % Longest path adjusted least; 
            [val_temp,ind_temp]=sort(adjst_traject_len);
            cluster_temp=cluster_temp(ind_temp);
            level_adjst_temp=[1:cluster_size]*traject_vert_dist; 
            level_adjst(cluster_temp,ind_l)=level_adjst_temp; 
        end
    end
    %% Adjusting Trajectories to Sub-space %% 
    for ind_ad=1:traject_size
        [link_ind,node_ind]=traject_level{ind_ad,:};
        node_ind_uni=unique(node_ind);
        node_len=length(node_ind_uni);
        if isempty(node_ind_uni) 
            continue; 
        else
            for ind_adn=1:node_len
                traject_node_ind=find(traject_regul{link_ind,3}==node_ind_uni(ind_adn));
                traject_free_adj{link_ind,2}(3,traject_node_ind)=traject_free_adj{link_ind,2}(3,traject_node_ind)+level_adjst(ind_ad,ind_l);
            end
        end
    end
end
%% Plotting %% 
traject_free_adj_heli=traject_free_adj;
v=figure (30);
% mesh(cubic_urban_map);
% plot3(grid_network(:,1),grid_network(:,2),grid_network(:,3),'.');
hold on; 
for ind_plt=1:traject_num 
    traj_temp=traject_free_adj{ind_plt,2};
    if traject_free_adj{ind_plt,2}(3,1) > min(air_level) 
        traj_temp=cat(2,[traj_temp([1:2],1);min(air_level)],traj_temp);
    end
    if traject_free_adj{ind_plt,2}(3,end) > min(air_level)
        traj_temp=cat(2,traj_temp,[traj_temp([1:2],end);min(air_level)]);
    end
    traj_temp_diff=sum(abs(diff(traj_temp,1,2))); 
    traj_temp(:,traj_temp_diff==0)=[];
    traject_free_adj_heli{ind_plt,2}=traj_temp;
    plot3(traj_temp(1,:),traj_temp(2,:),traj_temp(3,:),'-');
%     plot3(traject_free{ind_plt,2}(1,:),traject_free{ind_plt,2}(2,:),traject_free{ind_plt,2}(3,:),'-');
end
[urban_map_row,urban_map_col]=size(cubic_urban_map);
for ind_sub=1: air_level_len 
    height_temp=air_level(ind_sub); 
    sub_height_temp=height_temp+sub_space_height; 
    plot3([1,1,urban_map_col,urban_map_col,1],[1,urban_map_row,urban_map_row,1,1],[height_temp,height_temp,height_temp,height_temp,height_temp]);
    plot3([1,1,urban_map_col,urban_map_col,1],[1,urban_map_row,urban_map_row,1,1],[sub_height_temp,sub_height_temp,sub_height_temp,sub_height_temp,sub_height_temp]);
    plot3([1,1],[1,1],[height_temp,sub_height_temp]); 
    plot3([1,1],[urban_map_row,urban_map_row],[height_temp,sub_height_temp]);
    plot3([urban_map_col,urban_map_col],[urban_map_row,urban_map_row],[height_temp,sub_height_temp]);
    plot3([urban_map_col,urban_map_col],[1,1],[height_temp,sub_height_temp]);
    plot3([1,1],[1,1],[height_temp,sub_height_temp]); 
end
plot3(takeoff_landing_point(1,:),takeoff_landing_point(2,:),takeoff_landing_point(3,:),'o');
hold off;
grid on; 
save('traject_free_adj_heli.mat','traject_free_adj_heli');
toc;