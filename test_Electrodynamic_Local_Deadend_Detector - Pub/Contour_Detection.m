function [cont_val, sector_set] = Contour_Detection (input,n)
%% Contour Detection %% 
figure (100)
% mesh(input);
% hold;
% input=pnt_map_output;
% input=weight_EC;
% interval=20;
% n=1;
[cont_val,~]=contour(input,n);
contour_x=cont_val(1,:);
contour_y=cont_val(2,:);
contour_len=length(contour_x);
ind_ini=1;
sector_set=[];
while (ind_ini <= contour_len)
    x_ini=contour_x(ind_ini);
    y_ini=contour_y(ind_ini);
    sec_start=ind_ini+1;
    sec_end=y_ini+ind_ini;
    sector_ind=[sec_start;sec_end];
    sector_val=x_ini;
    sector_set=cat(2,sector_set,[sector_ind;sector_val]);
    ind_ini=sec_end+1;
end
close 100;
% figure (2)
% hold;
% % mesh(input);
% [~,sector_len]=size(sector_set);
% for ind_i=1:sector_len
%     sta_p=sector_set(1,ind_i);
%     end_p=sector_set(2,ind_i);
%     level=sector_set(3,ind_i);
%     cont_x=contour_x(sta_p:end_p);
%     cont_y=contour_y(sta_p:end_p);
%     cont_z=ones(size(cont_x))*level;
%     plot3(cont_x(1:interval:end),cont_y(1:interval:end),cont_z(1:interval:end),'b.','markersize',4);
% end
% % grid on;
% hold;
end