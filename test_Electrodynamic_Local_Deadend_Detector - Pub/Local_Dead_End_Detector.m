function [edge_set] = Local_Dead_End_Detector (input_val,cn_val_FW,cn_val_BW) 
%% Local_Dead_End_Detector %% 
% input_val=output_pt(3,:); 

smooth_val=input_val;
input_len=length(smooth_val);
forward_diff_smooth_val=[0,diff(smooth_val)]; 
backward_diff_smooth_val=fliplr([0,diff(fliplr(smooth_val))]); 
qual_flag=find(forward_diff_smooth_val==backward_diff_smooth_val); 
zero_flag1=find(forward_diff_smooth_val==0); 
zero_flag2=find(backward_diff_smooth_val==0); 
repeat_flag=setdiff(qual_flag,union(zero_flag1,zero_flag2));
if ~isempty(repeat_flag) 
    while ~isempty(repeat_flag) 
        for ind_i=2:input_len-1
            pre_val=smooth_val(ind_i-1);
            cur_val=smooth_val(ind_i);
            nex_val=smooth_val(ind_i+1);
            if cur_val ~= pre_val && cur_val ~= nex_val && pre_val == nex_val
                smooth_val(ind_i)=max(pre_val,nex_val);
            end
        end
        forward_diff_smooth_val=[0,diff(smooth_val)];
        backward_diff_smooth_val=fliplr([0,diff(fliplr(smooth_val))]);
        qual_flag=find(forward_diff_smooth_val==backward_diff_smooth_val);
        zero_flag1=find(forward_diff_smooth_val==0);
        zero_flag2=find(backward_diff_smooth_val==0);
        repeat_flag=setdiff(qual_flag,union(zero_flag1,zero_flag2));
    end
else 
end 
forward_diff_smooth_val=[0,diff(smooth_val)]; 
backward_diff_smooth_val=fliplr([0,diff(fliplr(smooth_val))]); 
edge1=intersect(find(forward_diff_smooth_val==cn_val_FW),find(backward_diff_smooth_val==-cn_val_FW)+1);    % edge 0 to 125 
edge2=intersect(find(forward_diff_smooth_val==cn_val_BW-cn_val_FW),find(backward_diff_smooth_val==cn_val_FW-cn_val_BW)+1);    % edge 125 to 255 
edge3=intersect(find(forward_diff_smooth_val==-cn_val_BW),find(backward_diff_smooth_val==cn_val_BW)+1);    % edge 255 to 0 
if isempty(edge2) || isempty(edge1) || isempty(edge3) 
    edge_set=[]; 
else
    edge_set=[min(edge1),round(mean(edge2)),max(edge3)]; 
end
% 
% figure (50)
% plot(smooth_val,'b'); 
% hold; 
% plot(forward_diff_smooth_val,'ro'); 
% plot(backward_diff_smooth_val,'gs'); 
% hold;
