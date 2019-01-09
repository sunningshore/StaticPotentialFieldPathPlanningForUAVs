%% test Layer Map %%
clear;
clc;
tic;
air_level=[15, 30, 45, 60, 75, 90, 105]; 
eval(['load(''cubic_urban_map_',num2str(min(air_level)),'.mat'',''cubic_urban_map'');']);
base_cubic_urban_map=cubic_urban_map;
weight_EC_cube=[]; 
map_img_rgb_cube=[];
cont_map_cube=[]; 
save('air_level.mat','air_level');
%% Generate Levels %% 
figure (10);
mesh(base_cubic_urban_map); 
hold; 
air_level_len=length(air_level);
contour_all_cube=[];
for ind_a=1:air_level_len
    building_height_thr=air_level(ind_a); 
    [weight_EC, map_img_rgb, cont_map, core_pt, bound_contour] = test_Contour_Map (building_height_thr); 
    if air_level(ind_a) == min(air_level) 
        disp('Here it comes! '); 
        core_pt_uni=core_pt; 
    end
    weight_EC_cube=cat(3,weight_EC_cube,weight_EC);
    map_img_rgb_cube=cat(3,map_img_rgb_cube,map_img_rgb);
    cont_map_cube=cat(3,cont_map_cube,cont_map);
    [~,contour_len]=size(bound_contour);
    eval(['contour_all_',num2str(air_level(ind_a)),'=[bound_contour;ones(1,contour_len)*air_level(ind_a)];']);
    eval(['contour_all_cube=cat(2,contour_all_cube,contour_all_',num2str(air_level(ind_a)),');']);
    plot3(bound_contour(1,:),bound_contour(2,:),ones(1,contour_len)*building_height_thr,'.');
end
[~,core_pt_len]=size(core_pt_uni);
% plot3(core_pt_uni(1,:),core_pt_uni(2,:),ones(1,core_pt_len)*min(air_level),'ro');
plot3(core_pt_uni(1,:),core_pt_uni(2,:),core_pt_uni(3,:),'ro');
hold;
save('core_pt_uni.mat','core_pt_uni'); 
save('weight_EC_cube.mat','weight_EC_cube');
save('map_img_rgb_cube.mat','map_img_rgb_cube');
save('cont_map_cube.mat','cont_map_cube');
save('contour_all_cube.mat','contour_all_cube');
%% Generate Mesh Grids %% 
grid_interval=5; 
grid_network = Grid_Generation (map_img_rgb_cube, air_level, grid_interval); 
save('grid_network.mat','grid_network');
%% Generate Trajectories %% 
traject_free=[];
traject_regul=[];
traject_orig=[];
figure (20)
mesh(cubic_urban_map);
hold on;
record_c1=[];  
for ind_c1=1:core_pt_len 
    record_c1=cat(2,record_c1,ind_c1); 
    for ind_c2=1:core_pt_len 
        if ind_c1 == ind_c2 || ismember(ind_c2,record_c1) 
            continue; 
        end
        orig_dest=[ind_c1;ind_c2];
        [pt_track, pt_track_orig] = test_Optimal_Trajectory_Finding3 (weight_EC_cube, map_img_rgb_cube, cont_map_cube, core_pt_uni, cubic_urban_map, contour_all_cube, air_level, orig_dest, grid_interval);
        [pt_track_regul, track_index]= test_Trajectory_Regulating (grid_network, pt_track); 
        og_pt=[ind_c1;ind_c2];
        traject_free_temp{1}=og_pt;
        traject_free_temp{2}=pt_track;
        traject_free=cat(1,traject_free,traject_free_temp);
        traject_regul_temp{1}=og_pt;
        traject_regul_temp{2}=pt_track_regul;
        traject_regul_temp{3}=track_index;
        traject_regul=cat(1,traject_regul,traject_regul_temp); 
        traject_orig_temp{1}=og_pt;
        traject_orig_temp{2}=pt_track_orig;
        traject_orig_temp{3}=track_index;
        traject_orig=cat(1,traject_orig,traject_orig_temp); 
%         plot3(pt_track_regul(1,:),pt_track_regul(2,:),pt_track_regul(3,:),'-r.');
        plot3(pt_track(1,:),pt_track(2,:),pt_track(3,:),'-r.');
    end
end
plot3(core_pt_uni(1,:),core_pt_uni(2,:),ones(1,core_pt_len)*min(air_level),'o');
% plot3(core_pt_uni(1,:),core_pt_uni(2,:),core_pt_uni(3,:),'o');
grid on;
hold off;
%% Index of Core Switch Points (Takeoff/Landing Point) %% 
takeoff_landing_pt=core_pt_uni;
takeoff_landing_pt(3,:)=air_level(1);
[takeoff_landing_pt_regul, takeoff_landing_pt_index]= test_Trajectory_Regulating (grid_network, takeoff_landing_pt); 
takeoff_landing_point=[takeoff_landing_pt_regul(1:3,:);takeoff_landing_pt_index];
save('traject_regul.mat','traject_regul');
save('traject_orig.mat','traject_orig');
save('traject_free.mat','traject_free');
save('takeoff_landing_point.mat','takeoff_landing_point');
toc; 