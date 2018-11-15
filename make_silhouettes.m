function [silhouettes, files] = make_silhouettes( T, Ncam, local_dir, Kdata)
% This function performs the simplest method of image segmentation:
% thresholding
% 
% [silhouettes, files] = make_silhouettes( T, Ncam, local_dir, Kdata)
% 
% Each pixel in an image is replaced with a white pixel if the image 
% intensity is less than some fixed constant T, or a black pixel if the 
% image intensity is greater than that constant.
% 
% T is the threshold value, Ncam is the number of images, local_dir is the
% directory where the images to be segmented are stored and Kdata are the
% values of the Kdata.txt file. 

% 1 grab all the image files
d = local_dir;
files = dir(fullfile(d, '*.png'));

% 2 load images
imgs = ones(2*Kdata(6),2*Kdata(3),Ncam);
for i=1:Ncam
    name = files(i).name;
    imgs(:,:,i) = rgb2gray(imread(strcat(d,name))); %Convert RGB image to grayscale (1 channel). These are not binary values yet.
end

% 3 compute silhouettes
silhouettes = NaN(size(imgs,1),size(imgs,2),Ncam);
for i=1:size(imgs,3)
     ch1 = imgs(:,:,i)<T; %chanel 1
     silhouettes(:,:,i) = ch1>0;
end

% 4 write the images and save them in current folder
for i = 1:Ncam
    imwrite(silhouettes(:,:,i),files(i).name)
end
end

