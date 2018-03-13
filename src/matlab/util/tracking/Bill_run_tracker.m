function [positions, time, imgs] = Bill_run_tracker( start_pos, target_sz, imgs, show_visualization )
% img_files = dir([video_path '\*.' file_type]);
% img_files = sort({img_files.name});

padding = 1.5;
kernel = struct('type','linear', 'sigma',0.2, 'poly_a',1, 'poly_b',7);
lambda = 1e-04;
output_sigma_factor = 0.1;
interp_factor = 0.075;

% Pixel shift -- larger is faster but coarser
cell_size = 1;
% Gray is faster; paper claims HOG is better when object is moving faster.
features = struct('gray',1, ...
                  'hog', 0, 'hog_orientations',8);

[positions, time, imgs] = Bill_Modified_tracker(imgs, start_pos, target_sz, ...
	padding, kernel, lambda, output_sigma_factor, interp_factor, cell_size, ...
	features, show_visualization);

end

