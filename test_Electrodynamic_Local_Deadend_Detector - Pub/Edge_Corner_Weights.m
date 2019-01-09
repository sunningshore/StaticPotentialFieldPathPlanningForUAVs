function weight_scal = Edge_Corner_Weights (beta,diam,alpha) 
% clear;
% clc;
% diam=1.9474*10;
% beta=1.9474*10;
% alpha=.85;

if alpha > 1 || alpha < 0 
    error('Alpha should be always within range (0,1)!'); 
end
diam=round(diam);
x=[0:.1:diam];
% x=linspace(1,diam);
% y=[0:.1:diam];
x_len=length(x);
if ~mod(x_len,2)
    x=[x,diam];
%     y=[y,diam];
end
% y=alpha.^x; 
y=(x+alpha*10).^(-2);
% y=(x).^(-2);
weight=zeros(diam*2+1,diam*2+1);
theta=[0:pi/50:2*pi];
t_len=length(theta);
for ind_r=1:x_len
    r=x(ind_r);
    w=y(ind_r);
%     xc=round(x_len/2)+r*cos(theta);
%     yc=round(x_len/2)+r*sin(theta);
    xc=r*cos(theta)+diam+1;
    yc=r*sin(theta)+diam+1;
    for ind_j=1:t_len 
%         if round(xc(ind_j))==0 || round(yc(ind_j))==0 
%             continue; 
%         else
            weight(round(xc(ind_j)),round(yc(ind_j)))=w;
%         end
    end
end
weight_uni=weight./max(max(weight));
weight_scal=weight_uni.*beta;
weight_scal(weight_scal==0)=min(weight_scal(weight_scal>0));
%% Plotting %% 
% [X,Y,Z] = cylinder(y,100); 
% mesh(X,Y,Z); 
% mesh(weight_scal);
% plot(x,y);
% end