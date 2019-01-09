function [weight_CN1,theta,search_step_fin,search_step_ini] = Connectivity_Weight_FW (start_pt,end_pt,input_map,bound_map,comp_contour,cn_val)
% start_pt;
% end_pt;
% input_map=weight_EC;
% bound_map=map_img_rgb;
% comp_contour=nearset_contour;
% cn_val=1; 
% figure (3)
% imshow(map_img_rgb);
% set(gca,'Ydir','normal');
% % mesh(weight_EC);
% hold;
% plot(start_pt(1),start_pt(2),'r*');
% plot(end_pt(1),end_pt(2),'ro');
% plot(comp_contour(1,:),comp_contour(2,:),'r.');

% tic;
alpha=2;
[map_row,map_col]=size(input_map);
R=sqrt(map_row^2+map_col^2);
cnw_z=[1:R].^alpha;
weight_CN1=zeros(map_row,map_col);
weight_bar=zeros(map_row,map_col);

% plot(1,map_row,'bs');

% search_step_fin=sqrt((end_pt(1)-comp_contour(1,:)).^2+(end_pt(2)-comp_contour(2,:)).^2);
% search_step_fin=round(min(search_step_fin));
search_step_fin=10;
search_step_ini=sqrt((end_pt(1)-start_pt(1,:)).^2+(end_pt(2)-start_pt(2,:)).^2);
% search_step_ini=sqrt((end_pt(1)-1).^2+(end_pt(2)-map_row).^2);
radius=search_step_ini;
map_edges=[[ones(1,map_row);1:map_row],[ones(1,map_row)*map_col;1:map_row],[1:map_col;ones(1,map_col)],[1:map_col;ones(1,map_col)*map_row]];
black_hole=comp_contour(3,1);

targ_slope=(start_pt(2)-end_pt(2))/(start_pt(1)-end_pt(1));
targ_degree=round(atand(targ_slope));
if end_pt(2) < start_pt(2) && end_pt(1) > start_pt(1) 
    targ_degree=targ_degree+180;
elseif end_pt(2) > start_pt(2) && end_pt(1) > start_pt(1) 
    targ_degree=targ_degree-180;
end

theta=[targ_degree-180:targ_degree+180];
theta_len0=length(theta);

theta_record=[];
theta_weight=[]; 
theta_gaus=[];
theta_gaus_dev=20; 

% [output_pt, bound_flag, simul_len] = Circle_Constructor (end_pt,radius,theta,input_map,bound_map);
% intersect_ind=find(output_pt(3,:)>black_hole); 
% bound_ind=find(bound_flag>0);
% intersect_ind=intersect(intersect_ind,bound_ind);
% intersect_len=length(intersect_ind); 

% dist_mat=sqrt((start_pt(1)-output_pt(1,intersect_ind)).^2+(start_pt(2)-output_pt(2,intersect_ind)).^2);
% [targ_val,targ_ind]=min(dist_mat);

% break_flag=zeros(1,intersect_len);
% break_flag([1,end])=[1,2];
% for ind_its=2:intersect_len-1
%     if intersect_ind(ind_its)-intersect_ind(ind_its-1)>3
%         break_flag(ind_its)=1;
%         break_flag(ind_its-1)=2;
%     end
% end
% sector_ind1=find(break_flag==1);
% sector_ind2=find(break_flag==2);
% if sector_ind1(1) > sector_ind2(1)
%     sector_ind2(1)=[];
% end
% sector_ind_len=min(length(sector_ind1),length(sector_ind2));
% sector_ind_end=sector_ind2(sector_ind2>targ_ind);
% sector_ind_end=intersect_ind(sector_ind_end(1));
% sector_ind_start=sector_ind1(sector_ind1<targ_ind);
% sector_ind_start=intersect_ind(sector_ind_start(end));

% track_num=1;
% intersect_ind_track=[sector_ind_start:sector_ind_end];
% track_len=length(intersect_ind_track);
% intersect_ind_track=cat(1,intersect_ind_track,ones(1,track_len)*track_num);
% output_pt_print=output_pt(:,intersect_ind_track(1,:));
% output_intersect_ind_track=intersect_ind_track;

% plot3(output_pt_print(1,:),output_pt_print(2,:),output_pt_print(3,:),'b.');
% plot3(output_pt(1,targ_ind),output_pt(2,targ_ind),output_pt(3,targ_ind),'bs');
% plot3(output_pt(1,intersect_ind),output_pt(2,intersect_ind),output_pt(3,intersect_ind),'bo');

% cn_weight=cnw_z(1);
% for ind_i=1:track_len
% %     weight_CN(output_pt_print(2,ind_i),output_pt_print(1,ind_i))=cn_weight;
%     weight_CN1(output_pt_print(2,ind_i),output_pt_print(1,ind_i))=cn_val;
% end
% termi_ind=weight_CN1([end_pt(2)-search_step_fin:end_pt(2)+search_step_fin],[end_pt(1)-search_step_fin:end_pt(1)+search_step_fin]); 
connect_bar=zeros(1,theta_len0);
iteration=0;
intersect_ind_track=1:361; 
intersect_ind_pre=intersect_ind_track;
% while isempty(find(termi_ind>0, 1))
for radius=search_step_ini:-1:search_step_fin
% while iteration < 50
    iteration=iteration+1;
%     radius=radius-1;
    [output_pt, bound_flag, simul_len] = Circle_Constructor (end_pt,radius,theta,input_map,bound_map); 
    cn_weight=cnw_z(iteration+1);
    cn_weight_vector=ones(1,simul_len)*cn_weight;
    %     intersect_ind=find(output_pt(3,:)<=black_hole | output_pt(3,:)==0);
    intersect_ind=find(output_pt(3,:)>black_hole); 
    bound_ind=find(bound_flag>0);
    intersect_ind=intersect(intersect_ind,bound_ind);
    intersect_len=length(intersect_ind);
    
    if intersect_len <=2
        continue; 
    end
    
    break_flag=zeros(1,intersect_len);
    break_flag([1,end])=[1,2];
    for ind_its=2:intersect_len-1
        if intersect_ind(ind_its)-intersect_ind(ind_its-1)>3 && intersect_ind(ind_its+1)-intersect_ind(ind_its)<3
            break_flag(ind_its)=1;
            if break_flag(ind_its-1)==0
                break_flag(ind_its-1)=2;
            end
        end
    end
    sector_ind1_ini=find(break_flag==1);
    sector_ind2_ini=find(break_flag==2);
    sector_ind1=sector_ind1_ini;
    sector_ind2=sector_ind2_ini;

    sector_len1=length(sector_ind1);
    sector_len2=length(sector_ind2);
    sector_ind_len=min(sector_len1,sector_len2);
    sector_num=0;
    intersect_ind_track_new=[];
    theta_gaus_temp=ones(1,theta_len0);
    for ind_set=1:sector_ind_len
        sector_temp=intersect_ind(sector_ind1(ind_set):sector_ind2(ind_set));
        temp_len=length(sector_temp);
%         corridor_ind=intersect(sector_temp,intersect_ind_track(1,:));
        corridor_ind=intersect(sector_temp,intersect_ind_pre);
        corridor_len=length(corridor_ind);
        if corridor_len > 1
            sector_num=sector_num+1;
            intersect_ind_track_new=cat(2,intersect_ind_track_new,[sector_temp;ones(1,temp_len)*sector_num]);
            theta_gaus_dev=length(sector_temp)/10; 
            theta_gaus_temp(sector_temp)=theta_gaus_temp(sector_temp).*normpdf(sector_temp,mean(corridor_ind),theta_gaus_dev);
%             connect_bar(sector_temp)=connect_bar(sector_temp)+cn_weight_vector(sector_temp);
        end
    end
    intersect_ind_pre=intersect_ind_track_new; 
    [~,print_len]=size(intersect_ind_track_new);
%     theta_gaus=cat(1,theta_gaus,theta_gaus_temp);
%     theta_weight_temp=zeros(1,theta_len0); 
%     theta_weight_temp(intersect_ind_track_new(1,:))=100; 
%     theta_weight=cat(1,theta_weight,theta_weight_temp);
%     theta_record=cat(1,theta_record,mat2cell(intersect_ind_track_new(1,:),[1]));
    
    if print_len 
    output_pt_print=output_pt(:,intersect_ind_track_new(1,:));
    output_pt_print1=output_pt_print; 
    output_pt_print2=output_pt_print+[ones(1,print_len);zeros(1,print_len);zeros(1,print_len)];
    output_pt_print3=output_pt_print-[ones(1,print_len);zeros(1,print_len);zeros(1,print_len)];
    theta_print=theta(intersect_ind_track_new(1,:));
    cn_weight_vector_print=cn_weight_vector(intersect_ind_track_new(1,:));
    theta_len=length(theta_print);
    for ind_i=1:theta_len
        if weight_CN1(output_pt_print1(2,ind_i),output_pt_print1(1,ind_i)) == 0
%             weight_CN(output_pt_print(2,ind_i),output_pt_print(1,ind_i))=cn_weight_vector_print(ind_i);
            weight_CN1(output_pt_print1(2,ind_i),output_pt_print1(1,ind_i))=cn_val; 
%             weight_CN1(output_pt_print2(2,ind_i),output_pt_print2(1,ind_i))=cn_val; 
%             weight_CN1(output_pt_print3(2,ind_i),output_pt_print3(1,ind_i))=cn_val; 
        end
    end
    connect_bar(intersect_ind_track_new(1,:))=connect_bar(intersect_ind_track_new(1,:))+cn_weight_vector(intersect_ind_track_new(1,:));
    termi_ind=weight_CN1([end_pt(2)-search_step_fin:end_pt(2)+search_step_fin],[end_pt(1)-search_step_fin:end_pt(1)+search_step_fin]);
    end
%     plot3(output_pt_print(1,:),output_pt_print(2,:),output_pt_print(3,:),'b.');
    
%     track_len_new=length(intersect_ind_track_new(1,:));
%     if track_len_new < track_len 
%         track_len=track_len_new;
%         output_intersect_ind_track=intersect_ind_track_new;
%     end
%     intersect_ind_track=intersect_ind_track_new;
end
connect_bar_uniform=connect_bar./max(connect_bar);
% theta_track=theta_gaus.*theta_weight; 

% connect_bar_diff=diff(connect_bar);
% zero_pt_flag=zeros(1,theta_len-1);
% for ind_d=2:theta_len-2
%     if connect_bar_diff(ind_d)<=0 && connect_bar_diff(ind_d-1)>0
%         zero_pt_flag(ind_d)=1;
%     end
% end
% maxima_ind=find(zero_pt_flag==1);
% maxima_val=connect_bar(maxima_ind);
% [maxima_sort_val,maxima_sort_ind]=sort(maxima_val,'descend');
% targ_ind=maxima_ind(maxima_sort_ind(1));
% % targ_ind=cat(2,targ_ind(1),round(mean(targ_ind)),targ_ind(2));
% if targ_ind(1) > 1
%     targ_ind=cat(2,1,targ_ind);
% end
% if targ_ind(end) < simul_len
%     targ_ind=cat(2,targ_ind,simul_len);
% end
% 
% connect_bar_interp = interp1(targ_ind,connect_bar(targ_ind),[1:simul_len],'linear');
% connect_bar_interp_uniform=connect_bar_interp./max(connect_bar_interp);

% radius_set=[radius:search_step_ini];
% radius_set_len=length(radius_set);
% for ind_r=1:radius_set_len 
%     r=radius_set(ind_r); 
%     [output_pt, ~, simul_len] = Circle_Constructor (end_pt,r,theta,input_map,bound_map); 
%     for ind_i=1:simul_len
% %         weight_bar(output_pt(2,ind_i),output_pt(1,ind_i))=bar_weight_uniform(ind_i);
%         weight_bar(output_pt(2,ind_i),output_pt(1,ind_i))=connect_bar_interp_uniform(ind_i);
%     end
% end
% weight_CN_slope=weight_CN.*weight_bar;
% hold;
% figure (4)
% mesh(theta_weight);
% figure (5)
% mesh(theta_track);
% figure (6)
% mesh(weight_CN1);
% toc;