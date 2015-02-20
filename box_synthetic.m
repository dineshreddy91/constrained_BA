clear all;
close all;
clc;

% First set the various variables.
% Noise standard deviation.
noise_std	=	[1 2 3 4 5];												% Gaussian noise std in points.
dim_num		=	3;															% Dimension
num_pts		=	1000;														% number of points
seed = 123456789;
pts_scale	=	2.5;														% Scale points with this factor after generation.
noise_num	=	5;															% Index into noise_std
pts_on_cube	=	pts_scale * uniform_on_cube( dim_num, num_pts, seed );		% Generated random points on a cube.
sigma		=	noise_std( noise_num );
K			=	[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];			% Camera intrinsic parameters.
w3D 		=	10;															% Some threshold passed to ceres.

debug_flag	=	0;
debug_impts	=	0;
num_views	=	15;															% Number of camera views.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% IGNORE THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     if matlabpool('size') ==0
%         try
%             matlabpool open 2;
%         catch
%         end
%     end
% random points generator on a cube
% system('rm myFile.txt');
% load('cube.mat');
% for num=1:size(t,2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% IGNORE THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Random intilization of the trajectories  without rotations 
projection_matrices(:,:,1)	=	[1 0 0 0.5;0 1 0 0.2; 0 0 1 20  ;0 0 0 1];	% z=0
projection_matrices(:,:,2)	=	[1 0 0 1  ;0 1 0 0.4; 0 0 1 19.5;0 0 0 1];	% z=1
projection_matrices(:,:,3)	=	[1 0 0 1.5;0 1 0 0.6; 0 0 1 19  ;0 0 0 1];	% z=2
projection_matrices(:,:,4)	=	[1 0 0 2  ;0 1 0 0.8; 0 0 1 18.5;0 0 0 1];	% z=3
projection_matrices(:,:,5)	=	[1 0 0 2.5;0 1 0 1  ; 0 0 1 18  ;0 0 0 1];	% z=4
projection_matrices(:,:,6)	=	[1 0 0 3  ;0 1 0 1.2; 0 0 1 17.5;0 0 0 1];	% z=5
projection_matrices(:,:,7)	=	[1 0 0 3.5;0 1 0 1.4; 0 0 1 17  ;0 0 0 1];	% z=6
projection_matrices(:,:,8)	=	[1 0 0 4  ;0 1 0 1.6; 0 0 1 16.5;0 0 0 1];	% z=7
projection_matrices(:,:,9)	=	[1 0 0 4.5;0 1 0 1.8; 0 0 1 16  ;0 0 0 1];	% z=8
projection_matrices(:,:,10)	=	[1 0 0 5  ;0 1 0 2  ; 0 0 1 15.5;0 0 0 1];	% z=9
projection_matrices(:,:,11)	=	[1 0 0 5.5;0 1 0 2.2; 0 0 1 15  ;0 0 0 1];	% z=10
projection_matrices(:,:,12)	=	[1 0 0 6  ;0 1 0 2.4; 0 0 1 14.5;0 0 0 1];	% z=11
projection_matrices(:,:,13)	=	[1 0 0 6.5;0 1 0 2.6; 0 0 1 14  ;0 0 0 1];	% z=12
projection_matrices(:,:,14)	=	[1 0 0 7  ;0 1 0 2.8; 0 0 1 13.5;0 0 0 1];	% z=13
projection_matrices(:,:,15)	=	[1 0 0 7.5;0 1 0 3  ; 0 0 1 13  ;0 0 0 1];	% z=14

% view of the 3d box
if debug_flag
	scatter3( pts_on_cube(1, :), pts_on_cube(2, :), pts_on_cube(3, :) );
	title( 'Points generated on a cube' );
end
  
% loop for num of movements
for nv = 1:num_views
	% Move the box to be made visible to the camera.
	rec_points				=	fcn_transformPoints( ...
									projection_matrices(:, :, nv), pts_on_cube' );
	point3d_actual(:, :, nv)=	rec_points;
	point3d_noise(:, :, nv)	=	noise( rec_points, sigma );

	% project to image frame
	img_points				=	projectPoints( rec_points, K );
	point2d(:, :, nv)		=	img_points;

	if debug_flag & debug_impts
		scatter( img_points(:,1), img_points(:,2) );
		title( sprintf( 'Image points in view %d', nv ) );
	end
end

% Ideally we will work on the noisy points.
% Since the depthmaps we have produce noisy reconstructions.
point3d						=	point3d_noise;

% Now save data in a format that ceres understands.
% helper for pointObservedValue
pointCloud_helper			=	point3d(:, :, 1)';
pointCloud2d				=	point2d(:, :, 1)';
for nv = 2:num_views
	pointCloud_helper		=	[ pointCloud_helper point3d(:, :, nv)' ];
	pointCloud2d			=	[ pointCloud2d point2d(:, :, nv)' ];
end
[ndims, npts_all] 			=	size( pointCloud_helper );
pointCloud					=	zeros( ndims, num_pts );
pointCloud( 1:3, 1:num_pts )=	point3d(:, :, 1)';


% Finally collect all 2D-3D observations into one vector.
for nv = 1:npts_all
	point_observed(1, nv)	=	pointCloud2d(1, nv);
	point_observed(2, nv)	=	pointCloud2d(2, nv);
	point_observed(3, nv)	=	pointCloud_helper(1, nv);
	point_observed(4, nv)	=	pointCloud_helper(2, nv);
	point_observed(5, nv)	=	pointCloud_helper(3, nv);
	point_observed(6, nv)	=	-0.1;
end

% The code does this same job as the for loop below. Just verify once.
A		=	zeros( num_pts, num_views );
A(:)	=	1:( num_pts * num_views );
A		=	A';
% count	=	1;
% for nv1 = 1:num_views
%     for nv2 = 1:num_pts
%         A( nv1, nv2 )		=	count;
%         count				=	count+1;
%     end
% end

% Why is this done ? There is no need for icp since only the camera
% matrix needs to be inverted to get this transformation!
cameraRtC2W(:,:,1)			=	[eye(3),[0;0;0]];
for nv = 1:num_views-1 
	[R, T]					=	icp( point3d(:, : , nv)', point3d(:, :, nv+1 )' );
	R_1						=	cameraRtC2W(:, 1:3, 1) * R;
	T_1						=	cameraRtC2W(:, 4, 1) + R * T;
	cameraRtC2W(:, :, nv+1)	=	[ R_1, T_1 ];
end

point_obs_index				=	sparse(A);
save('synthetic_data.mat', 'K', 'cameraRtC2W', 'pointCloud', 'point_obs_index', 'point_observed', 'w3D');

%% bundle ajustment with constarints
% num_all=[0 50 100 200 300 400 500 600 700 800 900 1000];
constraints					=	[ 0 10 20 30 40 50 60 70 80 90 100 ];
num_constraints				=	length( constraints );

for citer = 1:num_constraints

	load( 'synthetic_data.mat' ); % Make sure all 3D parameters are reset every iteration.
	num_of_rand_points		=	constraints( citer );

	% These labels don't mean much for the current experimentation.
	global objectLabel;
	objectLabel.length 				=	0;
	objectLabel.objectRtO2W			=	zeros(3,4,objectLabel.length);
	objectLabel.objectSize			=	zeros(objectLabel.length,3);
	objectLabel.optimizationWeight	=	zeros(1,objectLabel.length);

	% Plot 3D points before doing bundle adjustment.
	figure(1); scatter3( pointCloud(1, 1:num_pts), pointCloud(2, 1:num_pts), pointCloud(3, 1:num_pts) ); title( 'Before Adjustment' );
	axis( [-5 5 -6 6 0 30] );

	[ cameraRtC2W1, pointCloud ]	=	bundleAdjustment2D3DBoxFile( cameraRtC2W, pointCloud, point_obs_index, ...
										point_observed, K, w3D, 7, num_of_rand_points);

	% Plot 3D points after doing bundle adjustment.
	figure(2); scatter3( pointCloud(1, 1:num_pts), pointCloud(2, 1:num_pts), pointCloud(3, 1:num_pts) ); title( 'After Adjustment' );
	axis( [-5 5 -6 6 0 30] );

	first							=	point3d_actual(:, :, 1)';
	second							=	pointCloud(:, 1:num_pts );
	err_reproj						=	0;
	for err	= 1:num_pts
		err_reproj					=	err_reproj + ( sumsqr( first(:, err) - second(:, err) ) );
	end
	error_1							=	[ sigma num_of_rand_points err_reproj ];
	error_all( citer, : )			=	err_reproj;

	% Finally append the error for the current iteration to a file
	dlmwrite( 'recon_error.log', error_1, '-append');
	pause;
end
