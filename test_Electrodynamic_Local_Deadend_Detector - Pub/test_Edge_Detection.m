function [bin_map,boundary_set,building_set] = test_Edge_Detection (urban_data,height_thr,enlarg_factor) 
%% test_Edge_Detection %% 
load(urban_data); 
building_height_thr=height_thr;
enlarg_factor=enlarg_factor; 
[~,build_num]=size(Buildings);
[~,bound_len]=size(Boundaries); 
%% Big Loop Boundaries %% 
bound_x=Boundaries(1,:)*enlarg_factor;
bound_y=Boundaries(2,:)*enlarg_factor;
col_bin_map=round(max(bound_x)-min(bound_x));
col_bin_map=round(col_bin_map+.2*col_bin_map);
row_bin_map=round(max(bound_y)-min(bound_y));
row_bin_map=round(row_bin_map+.2*row_bin_map);
input_x_bound=round(bound_x-min(bound_x)+100);
input_y_bound=round(bound_y-min(bound_y)+100);
bin_map=zeros(col_bin_map,row_bin_map);
%% Interpolation %% 
[x_set_bound,y_set_bound] = Edge_Interpolation (input_x_bound,input_y_bound); 
boundary_set=[x_set_bound;y_set_bound];
%% Building Boundaries %% 
output_x_build=[];
output_y_build=[];
for ind_i=1:build_num
    building=Buildings(ind_i);
    build_outline=building.pos;
    build_height=building.height;
    if build_height < building_height_thr
        continue; 
    else
        build_x=build_outline(1,:)*enlarg_factor-min(bound_x)+100;
        build_y=build_outline(2,:)*enlarg_factor-min(bound_y)+100;
        [x_set_build,y_set_build] = Edge_Interpolation (build_x,build_y); 
        output_x_build=cat(2,output_x_build,x_set_build);
        output_y_build=cat(2,output_y_build,y_set_build);
    end 
end 
%% Interpolation %% 
building_set=[output_x_build;output_y_build];