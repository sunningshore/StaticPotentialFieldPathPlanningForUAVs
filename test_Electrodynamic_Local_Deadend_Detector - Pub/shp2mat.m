%% shp2mat %% 
clear; 
clc; 
%% Boundary %% 
Boundary = shaperead('BoundCBD.shp'); 
Boundaries=[Boundary.Y([1:end,1]);Boundary.X([1:end,1])]; 
%% Buildings %% 
shapefile = shaperead('Buildings_CBD.shp'); 
[build_num,~]=size(shapefile); 
Buildings.pos=[]; 
Buildings.height=[]; 
figure (100) 
plot(Boundaries(2,:),Boundaries(1,:),':'); 
hold; 
for ind_i=1:build_num 
    shpLat_temp=shapefile(ind_i).Y; 
    shpLon_temp=shapefile(ind_i).X; 
    shpAlt_temp=shapefile(ind_i).height_m; 
    pos_temp=[shpLat_temp;shpLon_temp]; 
    Buildings(ind_i).pos=pos_temp; 
    Buildings(ind_i).height=shpAlt_temp; 
    plot(shpLon_temp,shpLat_temp); 
end 
hold; 
save('ScnCBD.mat','Buildings','Boundaries'); 
%% Plotting %% 
