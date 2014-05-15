% project: From 2D to 3D Body Pose Detection Using Sketches 
% author: Ren Ding
% The University of Adelaide, School of Computer Science

%API path configuration
disp('Path configuration...');
addpath src;
addpath models;
addpath mesh;
addpath cvx;

if isunix()
  addpath mex_unix;
elseif ispc()
  addpath mex_pc;
end
compile;
cvx_setup;
    
% load PARSE_model which has been trained by struct SVM
load('PARSE_model');

% load images, start to detect joints
disp('Load images, start to detect joints...');
imlist = dir('images/*.png');
for i = 1:length(imlist)
    % load and display image
    im = imread(['images/' imlist(i).name]);
    clf; imagesc(im); axis image; axis off; 
    %drawnow;
    
    % joints detect function
    tic; 
    boxes = detect_fast(im, model, min(model.thresh,-1));
    dettime = toc; % record cpu time
    boxes = nms(boxes, .1); % nonmaximal suppression
    colorset = {'g','g','y','m','m','m','m','y','y','y','r','r','r','r','y','c','c','c','c','y','y','y','b','b','b','b'};
    xy = joints(im, boxes(1,:),colorset,imlist(i).name); % show the best detection
    %fprintf('Joints detection took %.1f seconds\n',dettime);
    %disp(['Joints detection Image:',imlist(i).name,'    done!']);
    
    % adjust the joints manually
    update_joints = 'y';    
    while (lower(update_joints) == 'y')
        figure(20);clf; imagesc(im); axis image; axis off; hold on;
        basis = load('mocapReducedModel.mat');
        skel = basis.skel;
        plot2Dskeleton([xy;ones(1,size(xy,2))]',skel,1,'texton',0);
        plot(xy(1,:),xy(2,:),'ro');
        
        % select joint to be adjusted
        [x,y] = ginput(1);
        
        if ( (x > 0) & (x <= size(im,2)) & (y > 0) & (y <= size(im,1)) )
            
            [a b] = min(sum((repmat([x;y],1,size(xy,2)) - xy).^2,1));
            plot(xy(1,b),xy(2,b),'ro','MarkerSize',10);
            
            [x,y] = ginput(1);
            xy(1,b) = x;xy(2,b) = y;
            
            update_joints = 'y';
        else
            update_joints = 'n';
        end;
    end;
    
    
    % save the alignment detect joints
    sample='sample';
    save('sample.mat');
    
    %Edge detect,guess each body parts width
    disp('Edge detect,guess each body parts width ...');
    [grey_img,edges]=edge_detect(sample);
    %in cameraAndPose.m, 
    %set 
    %alphasq_low  = alphasq - 1e-3;
    %alphasq_high = alphasq + alphasq_high_e;
    %to constraint the camera shifting within a limited area
    alphasq_high_e=8;
    %in estimateWPCamera.m
    estimateCameraParam=0.7;
    
    %3D Reconstruction
    disp('Start to reconstruct...');
    [X, R, t,~,~] = recon3DPose(grey_img,edges,estimateCameraParam,alphasq_high_e,im,xy,'viz',1);


%     %[X, R, t] = recon3DPose(im,xy,'viz',1);
%     %res=zeros(2,10);
%     [grey_img,edges]=edge_detect(sample);
%     %for alphasq_high_e=1:10
%         %in cameraAndPose.m, 
%         %set 
%         %alphasq_low  = alphasq - 1e-3;
%         %alphasq_high = alphasq + alphasq_high_e;
%         %to constraint the camera shifting within a limited area
%         alphasq_high_e=8
%         %in estimateWPCamera.m
%         estimateCameraParam=0.7;
%         [X, R, t,res1,res2] = recon3DPose(grey_img,edges,estimateCameraParam,alphasq_high_e,im,xy,'viz',1);
%         %res(1,alphasq_high_e)=res1;
%         %res(2,alphasq_high_e)=res2;
%     %end
    
end

disp('done');
