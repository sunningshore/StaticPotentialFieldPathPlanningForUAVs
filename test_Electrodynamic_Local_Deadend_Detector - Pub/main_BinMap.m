%% Binary Map Generator %% 
clear;
clc;
tic;
format long g;
%% Bin-Map Generation %% 
enlarg_factor=10^5;    % GPS2Field transform factor 
load('ScnCBD.mat');    % QGIS buildings properties 
building_height_thr_set=[15, 30, 45, 60, 75, 90, 105];    % Building height threshold, only calculate buildings higher than each threshold 
height_len=length(building_height_thr_set); 
[~,build_num]=size(Buildings);
[~,bound_len]=size(Boundaries); 
guard_dist=3;    % Minimum distance (m) allowed between UAV trajectories and buildings 
guardDistGPS=[1.279510067843056, 1.279510067843056; 103.8500000461538, 103.8500180461538]; 
%% Big Loop Boundaries %%
bound_x=Boundaries(1,:)*enlarg_factor;    % Coordinate transform GPS2Field 
bound_y=Boundaries(2,:)*enlarg_factor;
col_bin_map=round(max(bound_x)-min(bound_x));
col_bin_map=round(col_bin_map+.2*col_bin_map);
row_bin_map=round(max(bound_y)-min(bound_y));
row_bin_map=round(row_bin_map+.2*row_bin_map);
bound_x_rescale_ini=round(bound_x-min(bound_x)+100);
bound_y_rescale_ini=round(bound_y-min(bound_y)+100);
bin_map=zeros(col_bin_map,row_bin_map); 
% guard_dist transform GPS2Field 
guardDistWeight_x=guardDistGPS(1,:)*enlarg_factor-min(bound_x)+100;
guardDistWeight_y=guardDistGPS(2,:)*enlarg_factor-min(bound_y)+100;
guardDistWeightDist=sqrt((diff(guardDistWeight_x).^2)+(diff(guardDistWeight_y).^2)); 
% 30m - 27 (30m in World coordinate is 27 grids on field); 20m - 18; 10m - 9; 3m - 1.8; 
%% Generating Core Switch Points (Geo-centers of wide open areas in city) %% 
for ind_htr=1:height_len
    maxLat=0;
    minLat=col_bin_map;
    maxLon=0;
    minLon=row_bin_map;
    building_height_thr=building_height_thr_set(ind_htr); 
    %% Stuffing Operation Areas %%
    bin_map_bound = Filling_Enclosed_Areas (bin_map,bound_x_rescale_ini,bound_y_rescale_ini,255);    % Drawing city boundaries s on binary map (255)
    map_bound = Filling_Enclosed_Areas (bin_map,bound_x_rescale_ini,bound_y_rescale_ini,0);    % Drawing boundaries on 3D map 
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
        build_x=build_outline(1,:)*enlarg_factor-min(bound_x)+100;    % Coordinate transform GPS2Field 
        build_y=build_outline(2,:)*enlarg_factor-min(bound_y)+100;
        bin_map_build = Filling_Enclosed_Areas (bin_map_build,build_x,build_y,0);    % Drawing buildings on binary map (0)
        map_build = Filling_Enclosed_Areas (map_build,build_x,build_y,build_height);    % Drawing boundaries on 3D map 
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
    % Generating bin and 3D city maps 
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