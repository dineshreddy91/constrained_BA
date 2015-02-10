  clear all;
  close all;
  clc;
  
  %% random points generator on a cube
  dim_num = 3;
  n = 1000;% number of points
  seed = 123456789;
  [x] = uniform_on_cube ( dim_num, n, seed );
  x=x*2.5;
  sigma=0.5;
  %% random intilization of the trajectories  without rotations 
  rotation_matrices(:,:,1)=[1 0 0 0.5;0 1 0 0.2; 0 0 1 20;0 0 0 1]; % z=0
  rotation_matrices(:,:,2)=[1 0 0 1;0 1 0 0.4; 0 0 1 19.5;0 0 0 1]; % z=1
  rotation_matrices(:,:,3)=[1 0 0 1.5;0 1 0 0.6; 0 0 1 19;0 0 0 1]; % z=2
  rotation_matrices(:,:,4)=[1 0 0 2;0 1 0 0.8; 0 0 1 18.5;0 0 0 1]; % z=3
  rotation_matrices(:,:,5)=[1 0 0 2.5;0 1 0 1; 0 0 1 18;0 0 0 1];   % z=4
  rotation_matrices(:,:,6)=[1 0 0 3;0 1 0 1.2; 0 0 1 17.5;0 0 0 1]; % z=5
  rotation_matrices(:,:,7)=[1 0 0 3.5;0 1 0 1.4; 0 0 1 17;0 0 0 1]; % z=6
  rotation_matrices(:,:,8)=[1 0 0 4;0 1 0 1.6; 0 0 1 16.5;0 0 0 1]; % z=7
  rotation_matrices(:,:,9)=[1 0 0 4.5;0 1 0 1.8; 0 0 1 16;0 0 0 1]; % z=8
  rotation_matrices(:,:,10)=[1 0 0 5;0 1 0 2; 0 0 1 15.5;0 0 0 1];  % z=9
  rotation_matrices(:,:,11)=[1 0 0 5.5;0 1 0 2.2; 0 0 1 15;0 0 0 1]; % z=10
  rotation_matrices(:,:,12)=[1 0 0 6;0 1 0 2.4; 0 0 1 14.5;0 0 0 1]; % z=11
  rotation_matrices(:,:,13)=[1 0 0 6.5;0 1 0 2.6; 0 0 1 14;0 0 0 1]; % z=12
  rotation_matrices(:,:,14)=[1 0 0 7;0 1 0 2.8; 0 0 1 13.5;0 0 0 1]; % z=13
  rotation_matrices(:,:,15)=[1 0 0 7.5;0 1 0 3; 0 0 1 13;0 0 0 1]; % z=14
  
%  %% random intilization of the trajectories  with rotations 
%   rotation_matrices(:,:,1)=[1 0 0 0;0 1 0 0; 0 0 1 20;0 0 0 1]; % z=0
%   rotation_matrices(:,:,2)=[1 0 0 0;0 1 0 0; 0 0 1 19.5;0 0 0 1]; % z=1
%   rotation_matrices(:,:,3)=[1 0 0 0;0 1 0 0; 0 0 1 19;0 0 0 1]; % z=2
%   rotation_matrices(:,:,4)=[1 0 0 0;0 1 0 0; 0 0 1 18.5;0 0 0 1]; % z=3
%   rotation_matrices(:,:,5)=[1 0 0 0;0 1 0 0; 0 0 1 18;0 0 0 1]; % z=4
%   rotation_matrices(:,:,6)=[1 0 0 0;0 1 0 0; 0 0 1 17.5;0 0 0 1]; % z=5
%   rotation_matrices(:,:,7)=[1 0 0 0;0 1 0 0; 0 0 1 17;0 0 0 1]; % z=6
%   rotation_matrices(:,:,8)=[1 0 0 0;0 1 0 0; 0 0 1 16.5;0 0 0 1]; % z=7
%   rotation_matrices(:,:,9)=[1 0 0 0;0 1 0 0; 0 0 1 16;0 0 0 1]; % z=8
%   rotation_matrices(:,:,10)=[1 0 0 0;0 1 0 0; 0 0 1 15.5;0 0 0 1]; % z=9
%   rotation_matrices(:,:,11)=[1 0 0 0;0 1 0 0; 0 0 1 15;0 0 0 1]; % z=10
%   rotation_matrices(:,:,12)=[1 0 0 0;0 1 0 0; 0 0 1 14.5;0 0 0 1]; % z=11
%   rotation_matrices(:,:,13)=[1 0 0 0;0 1 0 0; 0 0 1 14;0 0 0 1]; % z=12
%   rotation_matrices(:,:,14)=[1 0 0 0;0 1 0 0; 0 0 1 13.5;0 0 0 1]; % z=13
%   rotation_matrices(:,:,15)=[1 0 0 0;0 1 0 0; 0 0 1 13;0 0 0 1]; % z=14
%   
  %% view of the 3d box
  %scatter3(x(1,:),x(2,:),x(3,:));
  
  %% loop for num of movements
  number_of_views=size(rotation_matrices,3);
  for num=1:number_of_views
  %% move the box to be made visible to camera
  rec_points=fcn_transformPoints(rotation_matrices(:,:,num),x');
  Point3d_actual(:,:,num)=rec_points;
  Point3d_noise(:,:,num)=noise(rec_points,sigma);

  %cameraRtC2W(:,:,num)=rotation_matrices(1:3,:,num);
  %% project to image frame
  K=[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];
  img_points=projectPoints(rec_points,K);
  Point2d(:,:,num)=img_points;
  %scatter(img_points(:,1),img_points(:,2));
  %ginput(1);
  end
  Point3d=Point3d_noise;
  

 %% data save for ceres
 
 % helper for pointObservedValue
  for i=1:number_of_views
    if i==1
        pointCloud_helper = Point3d(:,:,i)';
    else
        pointCloud_helper = [pointCloud_helper Point3d(:,:,i)'];
    end
  end
[aa,bb] = size(pointCloud_helper);
pointCloud = zeros(aa,bb);
pointCloud(1:3,1:n) = Point3d(:,:,1)' ;

%%% helper loop, to create 3d points equivalent to pointCloud_helper
for i=1:number_of_views
    if i==1
        pointCloud2d = Point2d(:,:,i)';
    else
        pointCloud2d = [pointCloud2d Point2d(:,:,i)'];
    end
end

for i=1:size(pointCloud_helper,2)
    
    pointObservedValue(1,i) = pointCloud2d(1,i);
    pointObservedValue(2,i) = pointCloud2d(2,i);        
    pointObservedValue(3,i) = pointCloud_helper(1,i);
    pointObservedValue(4,i) = pointCloud_helper(2,i);
    pointObservedValue(5,i) = pointCloud_helper(3,i);
    pointObservedValue(6,i) = -0.1;
end

% w3D is some threshold. 
w3D = 10;
count=1;
for i=1:number_of_views
    for j=1:n
        A(i,j)=count;
        count=count+1;
    end
end

cameraRtC2W(:,:,1)=[eye(3),[0;0;0]];
for h=1:number_of_views-1 
[R,T]=icp(Point3d(:,:,h)',Point3d(:,:,h+1)');
R_1=cameraRtC2W(:,1:3,1)*R;
T_1=cameraRtC2W(:,4,1)+R*T;
cameraRtC2W(:,:,h+1)=[R_1,T_1];
end


pointObserved=sparse(A);
save('synthetic_data.mat', 'K', 'cameraRtC2W', 'pointCloud', 'pointObserved', 'pointObservedValue', 'w3D');


%% bundle ajustment with constarints 

load('synthetic_data.mat');
num_of_rand_points=100;
%load('bundle_data.mat');
global objectLabel;
objectLabel.length = 0;
objectLabel.objectRtO2W = zeros(3,4,objectLabel.length);
objectLabel.objectSize = zeros(objectLabel.length,3);
objectLabel.optimizationWeight = zeros(1,objectLabel.length);

[cameraRtC2W1,pointCloud] = bundleAdjustment2D3DBoxFile(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, K, w3D,7,num_of_rand_points);
scatter3(pointCloud(1,1:size(Point3d_actual,1)),pointCloud(2,1:size(Point3d_actual,1)),pointCloud(3,1:size(Point3d_actual,1)));
first=Point3d_actual(:,:,1)';
second=pointCloud(:,1:n);
error=0;
for err=1:n
    error=error+(sumsqr(first(:,1)-second(:,1)));
end
error