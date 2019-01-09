function [weight_CN3, weight_CN_FW, weight_CN_BW] = Connectivity_Weight7 (start_pt,end_pt,input_map,bound_map,comp_contour)
% start_pt;
% end_pt;

% bound_map=map_img_rgb;

% figure (30); 
% imshow(bound_map); 
% set(gca,'Ydir','normal');
% % title('Conbined'); 
% hold; 
% plot(comp_contour(1,:),comp_contour(2,:),'b.'); 

cn_val_FW=125;
cn_val_BW=255;
tic; 
[weight_CN_FW,theta_FW,r_fin_FW,r_ini_FW] = Connectivity_Weight_FW (start_pt,end_pt,input_map,bound_map,comp_contour,cn_val_FW); 
[weight_CN, weight_CN_BW,theta_BW,r_fin_BW,r_ini_BW] = Connectivity_Weight_BW (start_pt,end_pt,input_map,bound_map,comp_contour,cn_val_BW); 
weight_CN_Conb=max(weight_CN_FW,weight_CN_BW); 
[map_row,map_col]=size(weight_CN_FW); 
% weight_CN_Conb=zeros(map_row,map_col); 
% for ind_i=1:map_row 
%     for ind_j=1:map_col 
%         val1=weight_CN_FW(ind_i,ind_j); 
%         val2=weight_CN_BW(ind_i,ind_j); 
%         if val1 == 0 
%             weight_CN_Conb(ind_i,ind_j)=val2; 
%         elseif val2 == 0 
%             weight_CN_Conb(ind_i,ind_j)=val1; 
%         end
%     end
% end

weight_map=ones(map_row,map_col);
theta_len=length(theta_FW); 
dist_od=sqrt((end_pt(1)-start_pt(1,:)).^2+(end_pt(2)-start_pt(2,:)).^2);
% sample_mat=zeros(theta_len,ceil(r_ini_FW));
delta_theta=max(theta_FW)-180; 
theta=theta_FW-delta_theta; 
% for ind_t=35
for ind_t=1:theta_len
    theta_temp=theta(ind_t); 
    radius=r_ini_FW; 
    [output_pt, simul_len] = Stright_Line_Constructor_dot_slope (end_pt,theta_temp,radius,weight_CN_Conb); 
%     sample_mat(ind_t,:)=output_pt(3,:); 
    output_pt1=output_pt;
    output_pt2=output_pt+[ones(1,simul_len);zeros(1,simul_len);zeros(1,simul_len)]; 
    output_pt3=output_pt-[ones(1,simul_len);zeros(1,simul_len);zeros(1,simul_len)]; 
    input_val=output_pt(3,:); 
    [edge_set] = Local_Dead_End_Detector (input_val,cn_val_FW,cn_val_BW); 
    if ~isempty(edge_set) 
        LD_len=edge_set(3)-edge_set(1); 
        for ind_i=edge_set(1):edge_set(3) 
            weight_map([output_pt1(2,ind_i):output_pt1(2,ind_i)],[output_pt1(1,ind_i):output_pt1(1,ind_i)])=.5; 
            weight_map([output_pt2(2,ind_i):output_pt2(2,ind_i)],[output_pt2(1,ind_i):output_pt2(1,ind_i)])=.5; 
            weight_map([output_pt3(2,ind_i):output_pt3(2,ind_i)],[output_pt3(1,ind_i):output_pt3(1,ind_i)])=.5;
            weight_CN_BW([output_pt1(2,ind_i):output_pt1(2,ind_i)],[output_pt1(1,ind_i):output_pt1(1,ind_i)])=weight_CN_BW([output_pt1(2,ind_i):output_pt1(2,ind_i)],[output_pt1(1,ind_i):output_pt1(1,ind_i)])*.5;
            weight_CN_BW([output_pt2(2,ind_i):output_pt2(2,ind_i)],[output_pt2(1,ind_i):output_pt2(1,ind_i)])=weight_CN_BW([output_pt2(2,ind_i):output_pt2(2,ind_i)],[output_pt2(1,ind_i):output_pt2(1,ind_i)])*.5; 
            weight_CN_BW([output_pt3(2,ind_i):output_pt3(2,ind_i)],[output_pt3(1,ind_i):output_pt3(1,ind_i)])=weight_CN_BW([output_pt3(2,ind_i):output_pt3(2,ind_i)],[output_pt3(1,ind_i):output_pt3(1,ind_i)])*.5; 
        end
    end
%     plot3(output_pt(1,:),output_pt(2,:),output_pt(3,:),'.'); 
%     if isempty(edge_set) 
%     else
%         plot3(output_pt(1,edge_set(1)),output_pt(2,edge_set(1)),output_pt(3,edge_set(1)),'ro');
%         plot3(output_pt(1,edge_set(3)),output_pt(2,edge_set(3)),output_pt(3,edge_set(3)),'rs');
%     end
end
% hold;
weight_CN3=weight_CN.*weight_CN_BW; 
% origin=weight_CN3;
% sigma0=1*sqrt(2);
% scale=sigma0*sqrt(2);
% f=fspecial('gaussian',[1,floor(3*scale)],scale);
% L1=double(origin);
% L2=conv2(L1,f,'same');
% L3=conv2(L2,f','same');
% weight_CN_Gaus=L3;
% figure(40) 
% plot(sample_mat(ind_t,:));
% eval(['title(''',num2str(ind_t),''');']);
%% Plotting %% 
% figure (10); 
% mesh(weight_CN_FW);
% title('Forward'); 
% figure (20); 
% mesh(weight_CN_BW);
% title('Backward'); 
% figure (30); 
% mesh(weight_CN_Conb);
% title('Conbined'); 
% figure (50); 
% mesh(weight_map);
% toc; 
