%% Binary Map Generator %% 
clear;
clc;
tic;
format long g;
%% Bin-Map Generation %% 
enlarg_factor=10^5;
load('ScnCBD.mat');
building_height_thr_set=[15, 30, 45, 60, 75, 90, 105]; 
height_len=length(building_height_thr_set); 
[~,build_num]=size(Buildings);
[~,bound_len]=size(Boundaries); 
guard_dist=30; 
guardDistGPS=[1.279510067843056, 1.279510067843056; 103.8500000461538, 103.8500180461538]; 
%% Big Loop Boundaries %%
bound_x=Boundaries(1,:)*enlarg_factor;
bound_y=Boundaries(2,:)*enlarg_factor;
col_bin_map=round(max(bound_x)-min(bound_x));
col_bin_map=round(col_bin_map+.2*col_bin_map);
row_bin_map=round(max(bound_y)-min(bound_y));
row_bin_map=round(row_bin_map+.2*row_bin_map);
bound_x_rescale_ini=round(bound_x-min(bound_x)+100);
bound_y_rescale_ini=round(bound_y-min(bound_y)+100);
bin_map=zeros(col_bin_map,row_bin_map); 

guardDistWeight_x=guardDistGPS(1,:)*enlarg_factor-min(bound_x)+100;
guardDistWeight_y=guardDistGPS(2,:)*enlarg_factor-min(bound_y)+100;
guardDistWeightDist=sqrt((diff(guardDistWeight_x).^2)+(diff(guardDistWeight_y).^2)); 
% 30m - 27; 20m - 18; 10m - 9; 3m - 1.8; 
%% Test Core Switch Points %% 
for ind_htr=1:height_len
    maxLat=0;
    minLat=col_bin_map;
    maxLon=0;
    minLon=row_bin_map;
    building_height_thr=building_height_thr_set(ind_htr); 
    %% Stuffing Operation Areas %%
    bin_map_bound = Filling_Enclosed_Areas (bin_map,bound_x_rescale_ini,bound_y_rescale_ini,255);
    map_bound = Filling_Enclosed_Areas (bin_map,bound_x_rescale_ini,bound_y_rescale_ini,0);
%     figure (1)
    % imshow(bin_map_bound);
    % set(gca,'Ydir','normal');
    %% Building Boundaries %%
    BuildingsHeight=[Buildings.height]'; 
    BuildingsAboveLevel=Buildings(BuildingsHeight>=building_height_thr); 
    [~,BuildAboveLevelNum]=size(BuildingsAboveLevel);
    bin_map_build=bin_map_bound;
    map_build=map_bound;
    for ind_i=1:BuildAboveLevelNum
        building=BuildingsAboveLevel(ind_i);
        build_outline=building.pos;
        build_outline(:,end)=[];
        build_height=building.height; 
        build_x=build_outline(1,:)*enlarg_factor-min(bound_x)+100;
        build_y=build_outline(2,:)*enlarg_factor-min(bound_y)+100;
        bin_map_build = Filling_Enclosed_Areas (bin_map_build,build_x,build_y,0);
        map_build = Filling_Enclosed_Areas (map_build,build_x,build_y,build_height);
        [minLatTemp,minLatInd]=min(build_x);
        [maxLatTemp,maxLatInd]=max(build_x);
        [minLonTemp,minLonInd]=min(build_y);
        [maxLonTemp,maxLonInd]=max(build_y);
        if minLatTemp < minLat
            minLat = minLatTemp;
            minLatx=minLatTemp;
            minLaty=build_y(minLatInd);
        end
        if maxLatTemp > maxLat
            maxLat = maxLatTemp;
            maxLatx=maxLatTemp;
            maxLaty=build_y(maxLatInd);
        end
        if minLonTemp < minLon
            minLon = minLonTemp;
            minLony=minLonTemp;
            minLonx=build_x(minLonInd);
        end
        if maxLonTemp > maxLon
            maxLon = maxLonTemp;
            maxLony=maxLonTemp;
            maxLonx=build_x(maxLonInd);
        end
    end
    ScnBuildsEdgePoints=[minLatx,minLaty;maxLonx,maxLony;maxLatx,maxLaty;minLonx,minLony]; 
    urban_map=bin_map_build;
    cubic_urban_map=map_build;
    eval(['save(''ScnBuildsEdgePoints_',num2str(building_height_thr),'.mat'',''ScnBuildsEdgePoints'');']);
    eval(['save(''urban_map_',num2str(building_height_thr),'.mat'',''urban_map'');']);
    eval(['save(''cubic_urban_map_',num2str(building_height_thr),'.mat'',''cubic_urban_map'');']);
    %% Plotting %%
    figure (1)
    imshow(bin_map_build);
    set(gca,'Ydir','normal');
    set(gca,'Color','black','xtick',[],'ytick',[],'position',[0 0 1 1]);
    myStyle = hgexport('factorystyle');
    myStyle.Resolution = 1000;
    eval(['hgexport(gcf,''building_map_img_',num2str(building_height_thr),'.jpg'', myStyle, ''Format'', ''jpeg'');']);
end
toc;