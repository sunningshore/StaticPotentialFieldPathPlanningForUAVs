function [weight_EC, map_img_rgb, cont_map, core_pt_uni, contour_all] = test_Contour_Map (building_height_thr) 
%% test Contour Map %%
% clear;
% clc;
% tic;
%% Variables %%
guard_dist=2;
eval(['load(''ScnBuildsEdgePoints_',num2str(building_height_thr),'.mat'',''ScnBuildsEdgePoints'');']);
eval(['load(''urban_map_',num2str(building_height_thr),'.mat'',''urban_map'');']);
%% Layer Gradient & Edge Extraction %%
filename=urban_map;
[Gamp_map,map_img_rgb] = Edge_Corner_Detection (filename);
%% Edge & Corner Weighting %%
weight_EC = Map_Safety_Weighting (Gamp_map,10,50,.95);
[weight_row,weight_col]=size(weight_EC);
%% Contour Generation %%
[minVal,minInd]=min(weight_EC(:)); 
a=floor(minInd/weight_row); 
b=mod(minInd, weight_row); 
if ~b 
    minPos=[a, weight_row]; 
else 
    minPos=[a+1, b]; 
end 
ScnBuildsEdgePoints=round(ScnBuildsEdgePoints); 
[edgePointNum,~]=size(ScnBuildsEdgePoints); 
edgeWeightVal=zeros(edgePointNum,1); 
edgeWeightValLowest=zeros(edgePointNum,1); 
for ind_i=1:edgePointNum 
    center_point=ScnBuildsEdgePoints(ind_i,:); 
    edgeWeightVal(ind_i)=weight_EC(ScnBuildsEdgePoints(ind_i,2),ScnBuildsEdgePoints(ind_i,1));
    theta=[-180:1:+180];
    t_len=length(theta);
    weight_register=zeros(1,t_len);
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
        
        [simul_val, simul_len] = Stright_Line_Constructor_dot_slope (center_point,input_degree,guard_dist+1,weight_EC);
        simul_x=simul_val(1,:);
        simul_y=simul_val(2,:);
        simul_z=simul_val(3,:);
        
        simul_len_cn=guard_dist;
        cn_weight_z=zeros(1,simul_len_cn);
        for ind_n=simul_len_cn
           weight_z=weight_EC(simul_y(ind_n),simul_x(ind_n));
        end
        
        weight_register(ind_t)=sum(weight_z);
        
    end 
    
    edgeWeightValLowest(ind_i)=max(weight_register); 
    
end
ScnBuildsEdgePoints=cat(2,ScnBuildsEdgePoints,edgeWeightVal,edgeWeightValLowest); 

cont_level=[max(max(weight_EC)):-30:max(edgeWeightValLowest)];
[cont_val, sector_set] = Contour_Detection (weight_EC,cont_level);
contour_x=cont_val(1,:);
contour_y=cont_val(2,:);
[~,sector_len]=size(sector_set);
%% Highest Contour %%
all_level=sector_set(3,:);
all_level_uni=unique(all_level);
all_level_sort=sort(all_level_uni,'descend');
max_ind=find(all_level >= all_level_sort(1));
max_sector_set=sector_set(:,max_ind);
[~,max_sector_len]=size(max_sector_set);
%% Lowest Contour %%
min_ind=find(all_level==all_level_sort(end));
min_sector_set=sector_set(:,min_ind);
[~,min_sector_len]=size(min_sector_set);
all_level_sort=all_level_sort(end);
%% Core Key Point %% 
% figure (2)
% imshow(map_img_rgb);
% set(gca,'Ydir','normal');
% % mesh(weight_EC);
% hold;
core_pt=[]; 
for ind_i=1:max_sector_len
    sta_p=max_sector_set(1,ind_i);
    end_p=max_sector_set(2,ind_i);
    level=max_sector_set(3,ind_i);
    cont_x=contour_x(sta_p:end_p);
    cont_y=contour_y(sta_p:end_p);
    cont_z=ones(size(cont_x))*level;
%     plot(cont_x,cont_y,'r.');
    x_wide=max(cont_x)-min(cont_x);
    y_wide=max(cont_y)-min(cont_y);
    if x_wide > 3 || y_wide > 3
        core_x=mean(cont_x);
        core_y=mean(cont_y);
        core_z=mean(cont_z);
        core_pt=cat(2,core_pt,[core_x;core_y;core_z]);
    end
end
core_pt_uni=unique(core_pt','rows')';
% plot3(core_pt_uni(1,:),core_pt_uni(2,:),core_pt_uni(3,:),'ro');
% hold;
%% Edge Contour Generation %%
all_sector_set=min_sector_set;
all_contour_x=contour_x;
all_contour_y=contour_y;
%% Plot Contour %% 
% figure (2)
% % imshow(map_img_rgb);
% % set(gca,'Ydir','normal');
% mesh(weight_EC);
% hold;
[~,all_sector_len]=size(all_sector_set);
interval=1;
contour_all=[];
for ind_i=1:all_sector_len
    sta_p=all_sector_set(1,ind_i);
    end_p=all_sector_set(2,ind_i);
    level=all_sector_set(3,ind_i);
    cont_x=all_contour_x(sta_p:end_p);
    cont_y=all_contour_y(sta_p:end_p);
    cont_z=ones(size(cont_x))*level;
    eval(['contour_',num2str(ind_i),'=round([cont_x;cont_y;cont_z;ind_i*ones(size(cont_x))]);']);
    eval(['contour_all=cat(2,contour_all,contour_',num2str(ind_i),');']);
% %     plot3(cont_x(1:interval:end),cont_y(1:interval:end),cont_z(1:interval:end)*255,'b','markersize',4);
%     plot3(cont_x(1:interval:end),cont_y(1:interval:end),cont_z(1:interval:end)+255,'b','markersize',4);
end
% hold;
cont_map=zeros(size(weight_EC));
[~,contour_all_len]=size(contour_all); 
for ind_cont=1:contour_all_len
    cont_map(contour_all(2,ind_cont),contour_all(1,ind_cont))=255;
end
% toc;