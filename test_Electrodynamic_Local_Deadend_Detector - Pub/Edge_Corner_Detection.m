function [Gamp_map,map_img_rgb] = Edge_Corner_Detection (filename)
input=filename;
[file_dim,~]=size(input);
if file_dim==1
    % filename=['map_img.jpg'];
    map_img_rgb=imread(filename);
    % map_img_rgb=imread('map_img_test.jpg');
    map_img=rgb2gray(map_img_rgb);
    map_img(:,end)=[];
    map_img_double=double(map_img);
    % map_img_double(map_img_double>0)=255;
else 
    map_img_double=filename;
    map_img_rgb=filename;
    map_img=filename;
end

[Gx, Gy] = gradient(map_img_double);
Gamp=sqrt(Gx.^2+Gy.^2);
Gamp_uni=Gamp;
Gamp_uni=Gamp_uni.*map_img_double;    % Sharpening Edges from Gradient 
Gamp_uni=Gamp_uni./max(max(Gamp_uni));    % Uniform Gradient 
map_uni=map_img./max(max(map_img));     % Uniform Map 

map_uni_double=double(map_uni);
% map_uni_double(map_uni_double>0)=1;
Gamp_map=Gamp_uni+map_uni_double;     % Combine Gradient (Edges) and Point Map 
end