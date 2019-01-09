function grid_network = Grid_Generation (map_img_rgb_cube, air_level, grid_interval) 
grid_interval=grid_interval;
map_img_rgb_cube=map_img_rgb_cube;
air_level=air_level;

air_level_len=length(air_level);
[map_row,map_col,map_lev]=size(map_img_rgb_cube); 
grid_cube_orig=zeros(map_row,map_col,map_lev);
for ind_i=1:grid_interval:map_row 
    for ind_j=1:grid_interval:map_col 
        grid_cube_orig(ind_i,ind_j,:)=1;
    end
end
grid_cube=grid_cube_orig.*map_img_rgb_cube;
grid_ind=find(grid_cube==255);
grid_ind_len=length(grid_ind);
grid_z=zeros(grid_ind_len,1);
grid_y=zeros(grid_ind_len,1);
grid_x=zeros(grid_ind_len,1);
grid_z(:)=air_level(1);
for ind_a=1:air_level_len-1 
    grid_z(grid_ind>ind_a*map_row*map_col)=air_level(ind_a+1);
end
% grid_z(grid_ind>map_row*map_col)=air_level(2);
% grid_z(grid_ind>2*map_row*map_col)=air_level(3);
layer_flag=grid_ind(grid_ind>map_row*map_col);
while ~isempty(layer_flag)
    grid_ind(grid_ind>map_row*map_col)=grid_ind(grid_ind>map_row*map_col)-map_row*map_col;
    layer_flag=grid_ind(grid_ind>map_row*map_col);
end
grid_x=(grid_ind-1)/map_row+1;
grid_y=mod(grid_ind,map_row);
grid_y(grid_y==0)=map_row;
grid_network=[grid_x,grid_y,grid_z];