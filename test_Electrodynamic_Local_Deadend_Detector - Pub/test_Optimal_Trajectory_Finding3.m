function [pt_track_fit, pt_track_elev] = test_Optimal_Trajectory_Finding3 (weight_EC_cube, map_img_rgb_cube, cont_map_cube, core_pt_uni, cubic_urban_map, contour_all_cube, air_level, orig_dest, traject_fit_interv) 
%% test Optimal Trajectory Finding %% 
% tic; 
input_map=weight_EC_cube(:,:,1);    % Waypoint selection from lowest layer 
GradMap=input_map; 
GradMap(GradMap<0)=0; 
[Gmag,Gdir]=imgradient(GradMap); 
GmagUni=Gmag./(max(max(Gmag))); 
bound_map=map_img_rgb_cube(:,:,1);
ground_bound_map=map_img_rgb_cube(:,:,1); 
contour_map=cont_map_cube(:,:,1);
height_level=air_level(1);
cubic_urban_map=cubic_urban_map;
contour_all_cube=contour_all_cube;
core_node=core_pt_uni; 
guard_dist=2;    % Minimum distance (field grids) allowed between UAV trajectories and buildings 
level_cross_thr=0.5;    % Local space complexity over global space complexity, 
% exceeding which the path planner shall go upstairs and continue searching

orig_ind=orig_dest(1);
dest_ind=orig_dest(2);
air_level_len=length(air_level);
contour_all_ind=contour_all_cube(5,:);
for ind_air=1:air_level_len 
    contour_all_temp=contour_all_cube(:,contour_all_ind==air_level(ind_air));
    eval(['contour_all_',num2str(air_level(ind_air)),'=contour_all_temp;']);
end
eval(['contour_all=contour_all_',num2str(air_level(1)),';']);
ground_contour_all=contour_all;
contour_map_ave=mean(mean(contour_map));
[weight_row,weight_col]=size(input_map);
core_node=round(core_node);
[~,core_len]=size(core_node);
core_node=cat(1,core_node,ones(1,core_len)*height_level);

for ind_c1=orig_ind 
    start_pt=round(core_node(:,ind_c1)); 
    for ind_c2=dest_ind 
        if ind_c1 == ind_c2
            continue;
        end
        level_ind=1; 
        end_pt=round(core_node(:,ind_c2));
        pt_track=[start_pt,end_pt]; 
        [~,track_len]=size(pt_track);
        vector_orig=pt_track(:,end-1);
        vector_dest=pt_track(:,end);
        level=vector_orig(3);
        
        comp_contour=contour_all;
        nearset_contour=comp_contour; 
        
        terminal_slope=(vector_dest(2)-vector_orig(2))/(vector_dest(1)-vector_orig(1));
        terminal_degree=atand(terminal_slope);
        if vector_dest(2) > vector_orig(2) &&  vector_dest(1) < vector_orig(1)
            terminal_degree=terminal_degree+180;
        elseif vector_dest(2) < vector_orig(2) &&  vector_dest(1) < vector_orig(1)
            terminal_degree=terminal_degree+180;
        end
        
        [output_pt, simul_len, slope_degree] = Stright_Line_Constructor_2dot (start_pt,end_pt,input_map,1);
        simul_x=output_pt(1,:);
        simul_y=output_pt(2,:);
        simul_z=output_pt(3,:);
        
        dist2dest=sqrt(sum((start_pt(1:2)-end_pt(1:2)).^2));
        
        % Find intersections between lowest contours and the straight line
        % from planner's current waypoint to destination. 
        % If no intersection, just fly straight; else, go to next step. 
        comp_contour=round(comp_contour);
        [~,~,intersect_ind_all]=intersect([[simul_x;simul_y],[simul_x-guard_dist;simul_y],[simul_x+guard_dist;simul_y]]',contour_all(1:2,:)','rows');
        [~,~,intersect_ind]=intersect([simul_x;simul_y]',nearset_contour(1:2,:)','rows');
        intersect_ind_all=intersect_ind; 
        
        dist_mat=sqrt(sum((vector_orig(1:2)*ones(1,length(intersect_ind))-[nearset_contour(1,intersect_ind);nearset_contour(2,intersect_ind)]).^2,1));
        [min_dist_val,min_dist_ind]=min(dist_mat);
        intersect_ind=intersect_ind(min_dist_ind);
        
        vector_orig_vrt=vector_orig;
        search_step_mat=sqrt((vector_orig_vrt(1)-comp_contour(1,:)).^2+(vector_orig_vrt(2)-comp_contour(2,:)).^2);
        [search_step_min,search_step_ind]=min(search_step_mat);
        search_step=search_step_min;
        tan_pt=comp_contour(:,search_step_ind);
        
        % Avoid PingPong effects; if planner jumps back and forth for 3
        % times, go upstairs. 
        ping_pong_count=[];
        ping_pong_ind=[];
        ping_pong_flag=0;
        ping_pong_up_flag=0;
        ping_pong_rep=0;

        % If no intersection, just fly straight; else, go to next step.
        if ~isempty(intersect_ind_all) %|| dist2dest > min_step 
            disp('Bro, the two end points cannot meet each other since they are too far away. Magpie Bridge being built...');
            Connectivity_Weight_Generation=toc;
            theta_dev=90; 
            iteration=0; 
            local_cplx_set=0; 
            
            % Search waypoints iteratively until no intersections between lowest contours and the straight line
            % from planner's current waypoint to destination. 
            while ~isempty(intersect_ind_all) 
                loop_flag=isempty(intersect_ind_all);
                iteration=iteration+1
                vector_orig_vrt=vector_orig;
                
                %% Local Complexity Linear Integration %% 
                vicin_size=round(min(weight_row,weight_col)/30);
                [cont_Gamp_map,cont_map_rgb] = Edge_Corner_Detection (contour_map);
                local_cplx=mean(mean(cont_Gamp_map([vector_orig(2)-vicin_size:vector_orig(2)+vicin_size],[vector_orig(1)-vicin_size:vector_orig(1)+vicin_size])));
                local_cplx_set=cat(2,local_cplx_set,local_cplx);
                local_cplx_rate=local_cplx/contour_map_ave*100; 
                pre_pt_level_ind=level_ind;
                eval(['pre_pt_contour_all=contour_all_',num2str(air_level(pre_pt_level_ind)),';']);
                if local_cplx_rate >= level_cross_thr %|| search_step <= guard_dist 
                    while local_cplx_rate >= level_cross_thr %|| search_step <= guard_dist 
                        if level_ind < air_level_len
                            level_ind=level_ind+1;
                            input_map=weight_EC_cube(:,:,level_ind);
                            GradMap=input_map;
                            GradMap(GradMap<0)=0;
                            [Gmag,Gdir]=imgradient(GradMap);
                            GmagUni=Gmag./(max(max(Gmag))); 
                            bound_map=map_img_rgb_cube(:,:,level_ind);
                            contour_map=cont_map_cube(:,:,level_ind);
                            eval(['contour_all=contour_all_',num2str(air_level(level_ind)),';']);
                            height_level=air_level(level_ind);
                            
                            [cont_Gamp_map,cont_map_rgb] = Edge_Corner_Detection (contour_map);
                            local_cplx=mean(mean(cont_Gamp_map([vector_orig(2)-vicin_size:vector_orig(2)+vicin_size],[vector_orig(1)-vicin_size:vector_orig(1)+vicin_size])));
                            local_cplx_rate=local_cplx/contour_map_ave*100;
                        else
                            break; 
                        end
                    end
                else
                    eval(['contour_all=contour_all_',num2str(air_level(level_ind)),';']);
                end
                if length(ping_pong_count) > 2 && level_ind <= air_level_len 
                    ping_pong_neighb=ping_pong_count(end-1:end); 
                    if ping_pong_neighb(1)*ping_pong_neighb(2)==-1 
                        ping_pong_rep=ping_pong_rep+1; 
                        if ping_pong_rep >= 3 
                            if level_ind < air_level_len 
                                ping_pong_up_flag=1;
                                level_ind=level_ind+1;
                                input_map=weight_EC_cube(:,:,level_ind);
                                GradMap=input_map;
                                GradMap(GradMap<0)=0;
                                [Gmag,Gdir]=imgradient(GradMap);
                                GmagUni=Gmag./(max(max(Gmag)));
                                bound_map=map_img_rgb_cube(:,:,level_ind);
                                contour_map=cont_map_cube(:,:,level_ind);
                                eval(['contour_all=contour_all_',num2str(air_level(level_ind)),';']);
                                height_level=air_level(level_ind);
                            else
                                theta_dev=theta_dev-ping_pong_rep;
                            end
                        else
                            search_step=search_step*2^(-ping_pong_rep);
                        end
                        ping_pong_ind=cat(1,ping_pong_ind, iteration); 
                        ping_pong_flag=1; 
                        disp('Ping Pong Effect Occurred'); 
%                         continue; 
                    else
                        search_step=min_dist_val; 
                        ping_pong_rep=0; 
                    end
                end
                comp_contour=contour_all;
                nearset_contour=comp_contour; 
                
                % Bi-directional connectivity field, for local dead end
                % detection 
                [weight_CN, weight_CN_FW, weight_CN_BW] = Connectivity_Weight7 (start_pt,end_pt,input_map,bound_map,nearset_contour); 
                % Crescent constructor for next optimal waypoint selection 
                [simul_vrt, conn_z, theta0, simul_len_vrt] = Crescent_Constructor (vector_orig_vrt,search_step,terminal_degree,theta_dev,input_map,weight_CN);
                simul_vrt_x=simul_vrt(1,:);
                simul_vrt_y=simul_vrt(2,:);
                simul_vrt_z=simul_vrt(3,:);
                simul_vrt_z_up=simul_vrt_z+abs(min(simul_vrt_z));
                simul_vrt_z_uniform=simul_vrt_z_up/max(simul_vrt_z_up);
                conn_z_uni=conn_z/max(conn_z); 
                %% Distance Towards All Directions %% 
                % Find maximum possible distance to fly (before hitting lowest contours) towards all directions on connectivity field
                [dist_register_ini, cn_weight_register_ini, theta, t_len] = Circle_Angle_Weight (vector_orig_vrt,terminal_degree,theta_dev,input_map,nearset_contour,weight_CN);
                dist_register=dist_register_ini;
                cn_weight_register=cn_weight_register_ini;
                %% Direction Weight Seducing %% 
                % Longer possible distance has higher weight 
                [max_dir_val,max_dir_ind]=max(dist_register);
                dist_register_seduceing=1-abs(theta-theta(max_dir_ind))/max(abs(theta-theta(max_dir_ind)));
                dist_register=dist_register.*dist_register_seduceing;
                %% Direction Weight Averaging %% 
                % Averaging over neighbouring directions to avoid single
                % peaks 
                dist_register_ave=zeros(1,t_len);
                cn_weight_register_ave=zeros(1,t_len);
                ave_step=round(t_len/8);
                for ind_t=1:t_len
                    if ind_t <= ave_step
                        dist_register_ave(ind_t)=mean(dist_register(1:ave_step*2));
                        cn_weight_register_ave(ind_t)=mean(cn_weight_register(1:ave_step*2));
                    elseif ind_t >= t_len-ave_step
                        dist_register_ave(ind_t)=mean(dist_register(end-ave_step*2:end));
                        cn_weight_register_ave(ind_t)=mean(cn_weight_register(end-ave_step*2:end));
                    else
                        dist_register_ave(ind_t)=mean(dist_register(ind_t-ave_step:ind_t+ave_step));
                        cn_weight_register_ave(ind_t)=mean(cn_weight_register(ind_t-ave_step:ind_t+ave_step));
                    end
                end
                dist_register_ave_uniform=dist_register_ave/max(dist_register_ave);
                cn_weight_register_ave_uniform=cn_weight_register_ave/max(cn_weight_register_ave);
                %% Direction Choosing %% 
                
                % Selecting the optimal waypoint based on safety and
                % connectivity fields (conbination factors .5 & .5, should be application dependent)
                simul_vrt_z_weight=(5/10)*simul_vrt_z_uniform+(5/10)*cn_weight_register_ave_uniform;
                
                [intersect_val_oval,intersect_ind_oval]=max(simul_vrt_z_weight);
                
                ping_pong_thr=intersect_ind_oval; 
                if ping_pong_thr >= 150 
                    ping_pong_count=cat(1,ping_pong_count,1); 
                elseif ping_pong_thr <= 30 
                    ping_pong_count=cat(1,ping_pong_count,-1); 
                else
                    ping_pong_count=cat(1,ping_pong_count,0); 
                end 
                
                if abs(diff(intersect_ind_oval)) <= 3
                    next_ind_cand=find(simul_z < level);
                    next_ind=next_ind_cand(1);
                    next_pt=[simul_x( next_ind);simul_y( next_ind);simul_z( next_ind)];
                    disp('Optimization 1');
                else
                    disp('Optimization 2');
                    next_pt=round([mean([simul_vrt(1,intersect_ind_oval);simul_vrt(2,intersect_ind_oval)],2);simul_z(1)]);
                end
                next_pt=cat(1,next_pt,height_level);
                
                test_orig=vector_orig;
                test_dest=next_pt;
                test_level=test_orig(3); 
                test_contour=pre_pt_contour_all; 
                [test_output_pt, test_simul_len, slope_degree] = Stright_Line_Constructor_2dot (test_orig,test_dest,input_map,1);
                test_simul_x=test_output_pt(1,:);
                test_simul_y=test_output_pt(2,:);
                test_simul_z=test_output_pt(3,:);
                [~,~,test_intersect_ind_all]=intersect([[test_simul_x;test_simul_y],[test_simul_x;test_simul_y],[test_simul_x;test_simul_y]]',test_contour(1:2,:)','rows');
                if ~isempty(test_intersect_ind_all)
                    test_dist_mat=sqrt(sum((test_orig(1:2)*ones(1,length(test_intersect_ind_all))-[test_contour(1,test_intersect_ind_all);test_contour(2,test_intersect_ind_all)]).^2,1));
                    test_dist_mat(test_dist_mat>=guard_dist)=[]; 
                else
                    test_dist_mat=[]; 
                end
                
                % Test if intersections between lowest contours and the
                % straight section towards next selected waypoint; 
                % If yes, select one more intermediate waypoint. 
                if ~isempty(test_dist_mat)
                    disp('Intermediate Intersection Occured. ');
                    [test_min_dist_val,test_min_dist_ind]=min(test_dist_mat);
                    test_intersect_ind_all=test_intersect_ind_all(test_min_dist_ind);
                    test_search_step=test_min_dist_val;
                    
                    % Gradient Descend method 
                    CurGradMag=GmagUni(next_pt(2),next_pt(1));
                    CurGradDir=Gdir(next_pt(2),next_pt(1)); 
                    CurGradDirRot=CurGradDir; 
                    if slope_degree > 0 
                        if CurGradDirRot > slope_degree 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot > 0 && CurGradDirRot < slope_degree 
                            CurGradDirRot = -CurGradDirRot + slope_degree ; 
                        elseif CurGradDirRot < 0 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        end 
                    elseif slope_degree < 0 
                        if CurGradDirRot > 0 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot < slope_degree 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot > 0 && CurGradDirRot < slope_degree 
                            CurGradDirRot = -CurGradDirRot + slope_degree ; 
                        end 
                    end 
                    
                    if abs(CurGradDirRot) > 90 
                        CurGradMag = -CurGradMag; 
                    end
                    
                    search_step = search_step * (1+ CurGradMag); 
                    search_step = min(search_step, test_search_step);
                    
                    if pre_pt_level_ind < level_ind
                        input_map=weight_EC_cube(:,:,pre_pt_level_ind);
                        bound_map=map_img_rgb_cube(:,:,pre_pt_level_ind);
                        contour_map=cont_map_cube(:,:,pre_pt_level_ind);
                        eval(['contour_all=contour_all_',num2str(air_level(pre_pt_level_ind)),';']);
                    end
                else
                    pt_track=cat(2,pt_track(:,1:end-1),next_pt,pt_track(:,end));
                    [~,track_len]=size(pt_track);
                    
                    vector_orig=pt_track(:,end-1);
                    vector_dest=pt_track(:,end);
                    level=vector_orig(3);
                    
                    comp_contour=contour_all;
                    nearset_contour=comp_contour;
                    
                    terminal_slope=(vector_dest(2)-vector_orig(2))/(vector_dest(1)-vector_orig(1));
                    terminal_degree=atand(terminal_slope);
                    if vector_dest(2) > vector_orig(2) &&  vector_dest(1) < vector_orig(1)
                        terminal_degree=terminal_degree+180;
                    elseif vector_dest(2) < vector_orig(2) &&  vector_dest(1) < vector_orig(1)
                        terminal_degree=terminal_degree+180;
                    end
                    
                    [output_pt, simul_len, slope_degree] = Stright_Line_Constructor_2dot (vector_orig,vector_dest,input_map,1);
                    simul_x=output_pt(1,:);
                    simul_y=output_pt(2,:);
                    simul_z=output_pt(3,:);
                    
                    dist2dest=sqrt(sum((vector_orig(1:2)-vector_dest(1:2)).^2));
                    
                    [~,~,intersect_ind_all]=intersect([[simul_x;simul_y],[simul_x-guard_dist;simul_y],[simul_x+guard_dist;simul_y]]',contour_all(1:2,:)','rows');
                    
                    [~,~,intersect_ind]=intersect([simul_x;simul_y]',nearset_contour(1:2,:)','rows');
                    intersect_ind_all=intersect_ind; 

                    dist_mat=sqrt(sum((vector_orig(1:2)*ones(1,length(intersect_ind))-[nearset_contour(1,intersect_ind);nearset_contour(2,intersect_ind)]).^2,1));
                    [min_dist_val,min_dist_ind]=min(dist_mat);
                    intersect_ind=intersect_ind(min_dist_ind);
                    
                    % Gradient Descend method 
                    CurGradMag=GmagUni(next_pt(2),next_pt(1)); 
                    CurGradDir=Gdir(next_pt(2),next_pt(1)); 
                    CurGradDirRot=CurGradDir; 
                    if slope_degree > 0 
                        if CurGradDirRot > slope_degree 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot > 0 && CurGradDirRot < slope_degree 
                            CurGradDirRot = -CurGradDirRot + slope_degree ; 
                        elseif CurGradDirRot < 0 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        end 
                    elseif slope_degree < 0 
                        if CurGradDirRot > 0 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot < slope_degree 
                            CurGradDirRot = CurGradDirRot - slope_degree ; 
                        elseif CurGradDirRot > 0 && CurGradDirRot < slope_degree 
                            CurGradDirRot = -CurGradDirRot + slope_degree ; 
                        end 
                    end 
                    
                    if abs(CurGradDirRot) > 90 
                        CurGradMag = -CurGradMag; 
                    end
                    
                    % Search step size decided by gradient descend method 
                    if ping_pong_up_flag 
                        search_step=min_dist_val; 
                        theta_dev=90; 
                        ping_pong_up_flag =0; 
                    else
                        search_step = search_step * (1+ CurGradMag);
                    
                        search_step=min(search_step, min_dist_val);
                    end
                end
            
            end
        end
    end
end
%% Level Fitting %% 
pt_track_level=[pt_track([1,2,4],:)]; 
pt_track_level(:,ping_pong_ind-1)=[];
%% Level Crossing Elevator %% 
pt_level=pt_track_level(3,:); 
[~,pt_level_len]=size(pt_track_level); 
pt_level_diff=diff(pt_level); 
insert_ind0=find(pt_level_diff~=0);
if ismember (pt_level_len-1,insert_ind0) 
    insert_pt=[pt_track_level(1:2,insert_ind0(1:end-1));pt_track_level(3, insert_ind0(1:end-1)+1)]; 
    last_insert_pt=[pt_track_level(1:2,pt_level_len);pt_track_level(3, pt_level_len-1)]; 
    insert_pt=cat(2,insert_pt,last_insert_pt); 
else
    insert_pt=[pt_track_level(1:2,insert_ind0);pt_track_level(3, insert_ind0+1)];
end
%%% Insert Waypoints %%% 
pt_temp=[pt_track_level,insert_pt];
insert_ind=insert_ind0+1+[0:length(insert_ind0)-1];
insert_ind1=insert_ind; 
order_flag=[1:pt_level_len];
for ind_o=1: length(insert_ind) 
    order_flag(insert_ind0(ind_o)+1:end) = order_flag(insert_ind0(ind_o)+1:end)+1; 
end
order_flag=[order_flag,insert_ind1];
[reorder_val,reorder_ind]=sort(order_flag);
pt_track_elev=pt_temp(:,reorder_ind);
pt_track_elev=cat(2,pt_track_elev,pt_track([1,2,4],end));
[~,pt_track_elev_len]=size(pt_track_elev);
%% Trajectory Poly - Fitting %% 
pt_track_fit=[];
fit_interval=traject_fit_interv;
for ind_f=1:pt_track_elev_len-1
    x1=pt_track_elev(1,ind_f);
    x2=pt_track_elev(1,ind_f+1);
    y1=pt_track_elev(2,ind_f);
    y2=pt_track_elev(2,ind_f+1);
    z1=pt_track_elev(3,ind_f);
    z2=pt_track_elev(3,ind_f+1);
    if z1 == z2
        if abs(x1-x2) >= abs(y1-y2)
            if x1 > x2
                fit_x=[x1:-fit_interval:x2];
                if fit_x(end) > x2
                    fit_x=cat(2,fit_x,x2);
                end
                fit_ind=polyfit([x1,x2],[y1,y2],1);
                fit_y=round(polyval(fit_ind,fit_x));
            else
                fit_x=[x1:fit_interval:x2];
                if fit_x(end) < x2
                    fit_x=cat(2,fit_x,x2);
                end
                fit_ind=polyfit([x1,x2],[y1,y2],1);
                fit_y=round(polyval(fit_ind,fit_x));
            end
        else
            if y1 > y2
                fit_y=[y1:-fit_interval:y2];
                if fit_y(end) > y2
                    fit_y=cat(2,fit_y,y2);
                end
                fit_ind=polyfit([y1,y2],[x1,x2],1);
                fit_x=round(polyval(fit_ind,fit_y));
            else
                fit_y=[y1:fit_interval:y2];
                if fit_y(end) < y2
                    fit_y=cat(2,fit_y,y2);
                end
                fit_ind=polyfit([y1,y2],[x1,x2],1);
                fit_x=round(polyval(fit_ind,fit_y));
            end
        end
        fit_z=ones(1,length(fit_y))*z1;
        pt_track_fit=cat(2,pt_track_fit,[fit_x;fit_y;fit_z]);
    else
        continue;
    end
end
pt_track_fit=cat(2,pt_track_fit,pt_track_elev(:,end));
[~,fit_track_len]=size(pt_track_fit);
% Path_Selecting=toc