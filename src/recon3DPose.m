% [X, R, t] = function recon3DPose(xy,im,varargin)
%
% Inputs:   xy - [2 x 14] matrix of 2D joint locations
%           im - Input image
%           
%
%
% Outputs:  X  - [3 x 14] matrix of 3D joint locations.
%           R  - [3 x 3]  Relative Camera Rotation.
%           t  - [3 x 1]  Relative Camera translation.


function [X, R, t,res1,res2] = recon3DPose(grey_img,edges,param,alphasq_high_e,im,xy,varargin)
% [X, R, t] = recon3DPose(xy,im,varargin)

% Parse parameters.
[pose.skel, pose.BOMP, pose.mu, pose.lambda1,...
 pose.lamda2, pose.K, pose.numIter,...
 pose.numIters2, pose.tol1, pose.tol2, pose.ks,...
 pose.optType, pose.viz, pose.annoids,pose.numPoints] = process_options(varargin,...
                                'skel','',... 
                                'BOMP','',...
                                'mu'  ,'',...
                                'lambda2',0.01,...
                                'lambda1',0.01,...
                                'K', setK(size(im,2),size(im,1),2),...
                                'numIter', 20,...
                                'numIters2',30,...
                                'tol1', 500, ...
                                'tol2', 1, ...
                                'ks', 15, ...
                                'optType', 1, ...
                                'viz', 0,...
                                'annoids',1:15,...
                                'numPoints',15);
pose.im = im;
pose.xy = [xy; ones(1,size(xy,2))];

% Load default basis and skeleton
if(isempty(pose.BOMP)||isempty(pose.mu)||isempty(pose.skel))
    basis = load('mocapReducedModel.mat');
    pose.BOMP = basis.B;
    pose.mu   = basis.mu;
    pose.skel = basis.skel;
    pose.numPoints = length(pose.skel.tree);
    pose.annoids    = [1:length(pose.skel.tree)]; 
end

% Reconstruct camera and pose.
[camera, pose,res1,res2] = cameraAndPose(pose,alphasq_high_e,param);

% Assign outputs
X = pose.XnewR;
R = camera.R;
t = camera.t;

% Show aligned output
if(pose.viz)
%    load frontCam; use the same camera model as computed before
%    - this way, we have the correct projection for the cylinders/sphere
    Xnew1 = alignToCamera(pose.XnewR,camera.R,camera.t,R,t);
    %xy = projectIntoAffineCam(Xnew1,pose.K,camera.R,camera.t,camera.S,pose.skel);
    figure(5);clf;
    %mid raduis parameters: body,upper arms,low arms, uppper legs, low legs
    params_mid_raduis = zeros(1,5);
    
    %top bottom raduis parameters: body top/bottom,upper arms top/bottom,low arms top/bottom,
    %uppper legs top/bottom, low legs top/bottom
    params_tb_raduis = zeros(1,10);
    
    [gaussianMan,xy_mtx,idx_xy] = visualizeGaussianModel(pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,params_mid_raduis,params_tb_raduis);
    drawCam(R,t);
    
    %change to figure1(gray image)
    figure(1);
    
    %draw project back image
    all_idx = unique(idx_xy);
    
    %in our case, numOfBodyparts = 23
    [~,numOfBodyparts] = size(all_idx);
    
    
    adjust_res_edge_length = zeros(1,numOfBodyparts); 
    adjust_tb_results=zeros(numOfBodyparts,2);
    
    for i = all_idx
        kk = find(idx_xy == i);
        ih = convhull(xy_mtx(1,kk),xy_mtx(2,kk));
        pbx=xy_mtx(1,kk(ih));
        pby=xy_mtx(2,kk(ih));
       
        if(i==2 ||i==5 ||i==8 ||i==11 ||i==13 ||i==15 ||i==18 ||i==20 ||i==23)
                [p1,p2,p3,p4]= find_project_edge(pbx,pby);
                
                plot([p1(1) p2(1)], [p1(2) p2(2)], '-r','LineWidth',1);
                plot([p3(1) p4(1)], [p3(2) p4(2)], '-r','LineWidth',1);
                
                %body_part:
                %1:body,2:upper arms,3:low arms,4: uppper legs,5: low legs
 
                if(i==13) body_part=1;end
                %upper arms
                if(i==15 ||i==20) body_part=2;end
                %low arms
                if(i==18) 
                    body_part=3;
                    [adjust_res_edge_length(i),adjust_tb_results(i,:)]=adjust_edge_fixed_top(xy_mtx,idx_xy,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,grey_img,body_part,i,adjust_tb_results(15,2));
                end
                if(i==23) 
                    body_part=3;
                    [adjust_res_edge_length(i),adjust_tb_results(i,:)]=adjust_edge_fixed_top(xy_mtx,idx_xy,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,grey_img,body_part,i,adjust_tb_results(20,2));
                end
                %uppper legs
                if(i==2 ||i==8 ) body_part=4;end
                %low legs
                if(i==5)
                    body_part=5;
                    [adjust_res_edge_length(i),adjust_tb_results(i,:)]=adjust_edge_fixed_top(xy_mtx,idx_xy,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,grey_img,body_part,i,adjust_tb_results(2,2));     
                end
                if(i==11) 
                    body_part=5;
                    [adjust_res_edge_length(i),adjust_tb_results(i,:)]=adjust_edge_fixed_top(xy_mtx,idx_xy,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,grey_img,body_part,i,adjust_tb_results(8,2));
                end
                
                %adjust edges
                if(i~=5 && i~=11 && i~=18 && i~=23)
                [adjust_res_edge_length(i),adjust_tb_results(i,:)]=adjust_edge(xy_mtx,idx_xy,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,grey_img,body_part,i);
                end
        else
                plot(pbx, pby, '-w','LineWidth',1);
        end

    end;
    
    %disp(adjust_res_edge_length);
    %convert
    %1:body,2:upper arms,3:low arms,4: uppper legs,5: low legs
    %To:
    %13:body,15,20:upper arms,18,23:low arms,
    %2,8: uppper legs,5,11: low legs
    adjust_res_five=zeros(1,5);
    adjust_res_five(1)= adjust_res_edge_length(13);
    adjust_res_five(2)= (adjust_res_edge_length(15)+adjust_res_edge_length(20))/2;
    adjust_res_five(3)= (adjust_res_edge_length(18)+adjust_res_edge_length(23))/2;
    adjust_res_five(4)= (adjust_res_edge_length(2)+adjust_res_edge_length(8))/2;
    adjust_res_five(5)= (adjust_res_edge_length(5)+adjust_res_edge_length(11))/2;
    
    adjust_tb_five=zeros(1,10);
    adjust_tb_five(1)=adjust_tb_results(13,1);
    adjust_tb_five(2)=adjust_tb_results(13,2);
    adjust_tb_five(3)=(adjust_tb_results(15,1)+adjust_tb_results(20,1))/2;
    adjust_tb_five(4)=(adjust_tb_results(15,2)+adjust_tb_results(20,2))/2;
    adjust_tb_five(5)=(adjust_tb_results(18,1)+adjust_tb_results(23,1))/2;
    adjust_tb_five(6)=(adjust_tb_results(18,2)+adjust_tb_results(23,2))/2;
    adjust_tb_five(7)=(adjust_tb_results(2,1)+adjust_tb_results(8,1))/2;
    adjust_tb_five(8)=(adjust_tb_results(2,2)+adjust_tb_results(8,2))/2;
    adjust_tb_five(9)=(adjust_tb_results(5,1)+adjust_tb_results(11,1))/2;
    adjust_tb_five(10)=(adjust_tb_results(5,2)+adjust_tb_results(11,2))/2;
    
    %after adjusting edge, draw the 3D model again
    Xnew1 = alignToCamera(pose.XnewR,camera.R,camera.t,R,t);
    figure(5);clf;
    [gaussianMan,xy_mtx,idx_xy] = visualizeGaussianModel(pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,adjust_res_five,adjust_tb_five);
    drawCam(R,t);
    
    adjust_xy=projectIntoAffineCam(Xnew1,pose.K,camera.R,camera.t,camera.S,pose.skel);
    adjust_xy=[adjust_xy(1,:);adjust_xy(2,:)];
    
    %draw the projection
    %figure(3);
    for i = all_idx
        kk = find(idx_xy == i);
        ih = convhull(xy_mtx(1,kk),xy_mtx(2,kk));
        pbx=xy_mtx(1,kk(ih));
        pby=xy_mtx(2,kk(ih));
       
        get_texture(pbx,pby,im,i,adjust_xy,0);
    end
    
    [gaussianMan,xy_mtx,idx_xy] = modelwithTexture(im,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,adjust_res_five,adjust_tb_five);
    
    
    %change to original image,draw projection onto it
    imshow(im);
    
    shift_res = ones(1,9);
    shift_res_idx = 1;
    for i = all_idx
       
        kk = find(idx_xy == i);
        ih = convhull(xy_mtx(1,kk),xy_mtx(2,kk));
        pbx=xy_mtx(1,kk(ih));
        pby=xy_mtx(2,kk(ih));
         
        if(i==2 ||i==5 ||i==8 ||i==11 ||i==13 ||i==15 ||i==18 ||i==20 ||i==23)
            [p1,p2,p3,p4]= find_project_edge(pbx,pby);

            pbx_org=xy_mtx(1,kk);
            pby_org=xy_mtx(2,kk);
            org = [pbx_org;pby_org]';

            %only use p1 which are top circle projection edge
            [q,~]= size(org);
            shift_idx = 1;
            for j=1:q
                if (org(j,:)==p1)
                    shift_res(shift_res_idx)=j;
                    break;
                end
            end

            get_texture(pbx,pby,im,i,adjust_xy,shift_res(shift_res_idx));

            shift_res_idx = shift_res_idx+1;
        else
            %do not shift
            get_texture(pbx,pby,im,i,adjust_xy,0);
        end
    end
    %display the adjust results
    %disp(adjust_res_five);
    %disp(adjust_tb_five);
    %disp(shift_res);
    [gaussianMan,xy_mtx,idx_xy] = modelwithTexture(im,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,adjust_res_five,adjust_tb_five);

end
