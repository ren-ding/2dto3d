addpath src;
addpath models;
addpath mesh;

if isunix()
  addpath mex_unix;
elseif ispc()
  addpath mex_pc;
end

compile;

% load PARSE_model which has been trained by struct SVM
load('PARSE_model');

% load images, start to detect joints
% imlist = dir('images/*.png');
% for i = 1:length(imlist)
%     % load and display image
%     im = imread(['images/' imlist(i).name]);
%     clf; imagesc(im); axis image; axis off; 
%     %drawnow;
%     
%     % joints detect function
%     tic;
%     
%     boxes = detect_fast(im, model, min(model.thresh,-1));
%     dettime = toc; % record cpu time
%     boxes = nms(boxes, .1); % nonmaximal suppression
%     colorset = {'g','g','y','m','m','m','m','y','y','y','r','r','r','r','y','c','c','c','c','y','y','y','b','b','b','b'};
%     
%     xy = joints(im, boxes(1,:),colorset,imlist(i).name); % show the best detection
%     
    %loading samples with detected joints
    sample='sample6';
    load(sample);

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
    
    %fprintf('Joints detection took %.1f seconds\n',dettime);
    %disp(['Joints detection Image:',imlist(i).name,'    done!']);
    disp('Start to reconstruct...');
    
    %3D Reconstruction
    addpath('..\cvx\');
    cvx_setup;
    %[X, R, t] = recon3DPose(im,xy,'viz',1);
    res=zeros(2,10);
    [grey_img,edges]=edge_detect(sample);
    %for alphasq_high_e=1:10
        alphasq_high_e=8
        param=0.7;
        [X, R, t,res1,res2] = recon3DPose(grey_img,edges,param,alphasq_high_e,im,xy,'viz',1);
        res(1,alphasq_high_e)=res1;
        res(2,alphasq_high_e)=res2;
    %end
    %disp('press any key to continue');
    %pause;
    
%end

disp('done');
