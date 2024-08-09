clc
clear all

% Load variables: BackgroundPointCloudRGB,ForegroundPointCloudRGB,K,crop_region,filter_size)
load data.mat

% Set camera parameters for rectified stereo pair
K_left = [1550, 0, 1082.6879; 0, 1091.4844, 1043.0461; 0, 0, 1];
K_right = [1500, 0, 1300.3121; 0, 1091.4844, 1043.0461; 0, 0, 1];
R_rect = eye(3);
t_rect = [0.1; 0; -0.25];

data3DC = {BackgroundPointCloudRGB, ForegroundPointCloudRGB};

for step = 0:1
    tic
    fname_left = sprintf('SampleOutput%03d_left.jpg', step);
    fname_right = sprintf('SampleOutput%03d_right.jpg', step);
    display(sprintf('\nGenerating %s and %s', fname_left, fname_right));
    t = step * t_rect;
    M_left = K_left * [R_rect t];
    M_right = K_right * [R_rect (t - t_rect)];
    im_left = PointCloud2Image(M_left, data3DC, crop_region, filter_size/4);
    im_right = PointCloud2Image(M_right, data3DC, crop_region, filter_size/4);
    imwrite(im_left, fname_left);
    imwrite(im_right, fname_right);
    toc
end



% Compute disparity map using rectified stereo pair
img_left = imread('SampleOutput000_left.jpg');
img_right = imread('SampleOutput000_right.jpg');
img_left = rgb2gray(img_left);
img_right = rgb2gray(img_right);
disparity_range = [-64, 64];
disparity_map = disparity(img_left, img_right, 'BlockSize', 11, 'DisparityRange', disparity_range);
figure;
imshow(disparity_map, disparity_range);
title('Disparity Map');
%colormap(gca, jet);
%colorbar;


% Step 2
% Set stereo baseline and disparity range
baseline = 0.05; % 5 cm
min_depth = 2.5; % meters
max_depth = 10; % meters
disparity_range = round((K(1,1)*baseline)./([max_depth, min_depth]));

% Step 3
% Convert RGB image to grayscale
im_left_gray = rgb2gray(imread('SampleOutput000_left.jpg'));
im_right_gray = rgb2gray(imread('SampleOutput001_right.jpg'));

% Compute disparity map
%im_right_gray = imresize(im_left_gray, size(im_right_gray));
disparity_map = stereo_disparity(im_left_gray, im_right_gray, disparity_range);

% Display results
figure;
imshow(disparity_map, [disparity_range(1), disparity_range(2)]);
colormap grey
colorbar
title('Disparity Map');

%%%%%%%
function [disparity_map] = stereo_disparity(im_left, im_right, disparity_range)
% Compute the disparity map using stereo vision
% im_left and im_right are the left and right grayscale images

% Compute disparity map using block matching algorithm
disparity_map = zeros(size(im_left));
for i = 1:size(im_left,1)-disparity_range(1)+1
    for j = 1:size(im_left,2)-disparity_range(1)+1
        % Define search range for matching
        if j < disparity_range(1)
            search_range = [j, j+disparity_range(2)];
        elseif j > size(im_left,2)-disparity_range(2)
            search_range = [j-disparity_range(2), j];
        else
            search_range = [j-disparity_range(1), j+disparity_range(1)];
        end
        
        % Find best match in search range
        ssd_min = inf;
        best_match = j;
        for k = search_range(1):search_range(2)
            if k < 1 || k > size(im_left,2)
                continue;
            end
            % Calculate SSD between image patches
            % Ensure that the image patches being compared have the same size
            im_left_patch = im_left(i:i+disparity_range(1)-1,j:j+disparity_range(1)-1);
            im_right_patch = im_right(i:i+disparity_range(1)-1,k:k+disparity_range(1)-1);
            ssd = sum(sum((im_left_patch-im_right_patch).^2));
            if ssd < ssd_min
                ssd_min = ssd;
                best_match = k;
            end
        end
        
        % Compute disparity and store in disparity map
        disparity_map(i,j) = j-best_match;
    end
end
end

