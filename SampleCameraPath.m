clc
clear all
% load variables: BackgroundPointCloudRGB,ForegroundPointCloudRGB,K,crop_region,filter_size)
load data.mat

% Step 1: Modify the Rendering Code
% reduce image resolution by a factor of 4
K(1:2, :) = K(1:2, :) / 4;

% disable max_filter operation
filter_size = 0;

data3DC = {BackgroundPointCloudRGB,ForegroundPointCloudRGB};
R       = eye(3);
move    = [0 0 -0.25]';

% Step 2: Design Stereo Geometry and Disparity Search Parameters
% Stereo geometry
baseline = 0.1; % 10 cm
R = [1 0 0; 0 1 0; 0 0 1];
t = [baseline 0 0]';
M_left = K * [R t];
M_right = K * [R -t];

% Disparity search range
min_disp = 0;
max_disp = round(baseline / 0.02); % 2% of baseline


for step=0:1
    tic
    fname       = sprintf('SampleOutput%03d.jpg',step);
    display(sprintf('\nGenerating %s',fname));
    t           = step * move;
    M           = K*[R t];
    im          = PointCloud2Image(M,data3DC,crop_region,filter_size);
    
    % Step 3: Run Stereo
    % convert RGB image to grayscale
    im_gray = 0.3 * im(:,:,1) + 0.59 * im(:,:,2) + 0.11 * im(:,:,3);
    
    % compute disparity map
    disp_map = stereo(im_gray, M_left, im_gray, M_right, min_disp, max_disp);
    
    imwrite(disp_map,fname);
    toc    
end

% stereo function:-
% convert input images to double precision
left_image = im2double(left_image);
right_image = im2double(right_image);

% compute image dimensions
[height, width] = size(left_image);

% initialize disparity map
disparity_map = zeros(height, width);

% loop over all pixels in the left image
for i = 1:height
    for j = 1:width
        % compute search range for current pixel
        search_range = max(1, j - disparity_range):min(width, j + disparity_range);
        num_pixels = length(search_range);
        
        % initialize minimum SAD and corresponding disparity
        min_sad = inf;
        min_disp = 0;
        
        % loop over search range
        for k = 1:num_pixels
            % compute SAD between left and right image patches
            patch_left = left_image(max(1, i - 4):min(height, i + 4), max(1, j - 4):min(width, j + 4));
            patch_right = right_image(max(1, i - 4):min(height, i + 4), max(1, search_range(k) - 4):min(width, search_range(k) + 4));
            sad = sum(abs(patch_left(:) - patch_right(:)));
            
            % update minimum SAD and corresponding disparity
            if sad < min_sad
                min_sad = sad;
                min_disp = abs(j - search_range(k));
            end
        end
        
        % set disparity value in disparity map
        disparity_map(i, j) = min_disp;
    end
end

% normalize disparity map
disparity_map = disparity_map / disparity_range;


% stereo function ends

