function weight_EC = Map_Safety_Weighting (input,beta_factor,diam_factor,alpha)
%% Edge & Corner Weights %% 
% input=Gamp_map;    % Referrence for Edge Value 
% beta_factor=10;
% diam_factor=10;
% edge_val=1;
% alpha=.9;

Gamp_map_edge=input;    % Referrence for Edge Value 
Gamp_map_surf=input;    % Referrence for Surface Preserve 
Gamp_map_surf(Gamp_map_surf>1)=1;
Gamp_map_surf(Gamp_map_surf==1)=255;
[map_row,map_col]=size(input);    % Extracting Edges From the Map 
edge_ind=find(input>1.1);
edge_x=floor((edge_ind-1)./map_row)+1;
edge_y=mod(edge_ind,map_row);
edge_y(edge_y==0)=map_row; 
edge_len=length(edge_ind);
for ind_i=1:edge_len
    temp_x=edge_x(ind_i);
    temp_y=edge_y(ind_i);
    edge_val=Gamp_map_edge(temp_y,temp_x);
    weight = Edge_Corner_Weights (edge_val*beta_factor,edge_val*diam_factor,alpha); 
%     weight = Edge_Corner_Weighting (edge_val*beta_factor,edge_val*diam_factor); 
    [weight_row,weight_col]=size(weight);
    weight_len = (weight_row-1)/2;
    weight_range_y=[temp_y-weight_len:temp_y,temp_y+1:temp_y+weight_len];
    weight_range_x=[temp_x-weight_len:temp_x,temp_x+1:temp_x+weight_len];
    
    in_range_y_ind=find(weight_range_y >= 1 & weight_range_y<= map_row);
    in_range_x_ind=find(weight_range_x >= 1 & weight_range_x<= map_col);
    in_range_ind=intersect(in_range_x_ind,in_range_y_ind);
    
    in_range_y=weight_range_y(in_range_ind);
    in_range_x=weight_range_x(in_range_ind);
    weight_in_range=weight(in_range_ind,in_range_ind);
    Gamp_map_surf(in_range_y,in_range_x)=Gamp_map_surf(in_range_y,in_range_x)-weight_in_range;
end
weight_EC=Gamp_map_surf;
% weight_thr=max(max(weight_EC))*(10^0.1-1)/10^0.1;    % 4dB threshold, 10*log(max/(max-thresh))=4dB. 
% weight_thr=0;
% weight_EC(weight_EC < weight_thr)=0;
end