function [camera, pose,res1,res2] = cameraAndPose(pose,alphasq_high_e,param)
% Inputs:   pose - Structure containing 2D annotations and algorithm
%                  parameters. See recon3DPose.m
%
% Outputs:  camera - Structure with estimated camera parameters.
%           pose   - Structure with estimated 3D pose.
%

% Compute lengths and skeleton connectivity matrix.
pose = getLimbLengths(pose);

%% Initialization
% Scale input so that K = eye(3).
pose = scaleInput(pose);

% Reshape Basis
pose = reshapeBasis(pose);

% Estimate initial cameras
rigidInds = [1 2 5 8]; % Use rigid landmarks to estimate initial cam.
xyMissing = pose.xyh(:,rigidInds);
XMissing  = pose.Xmu(rigidInds,1:3);
param=0.8;
[camera]  = estimateWPCamera(xyMissing',XMissing,param);

if (0)
    % reset cameras
    load('new_camera6');
    camera=new_camera;

    camera.R = new_camera.R;
    camera.M = new_camera.S*[1 0 0;0 1 0]*new_camera.R;
    camera.Raff = [1 0 0;0 1 0]*new_camera.R;
end;

%camera.R=new_camera.R;
%camera.Raff=new_camera.Raff;

%Assemble projection matrix
Msub = kron(eye(length(pose.annoids),length(pose.annoids)),camera.M);
Mmu = Msub*pose.mureVis;
xyt = pose.xyvisible(:) - kron(ones(length(pose.annoids),1),camera.taff);
res = xyt - Mmu;

%% Projected Matching Pursuit
selectIdx = [];
pose.numIters = 0;
pose.repErr = inf;
while (pose.repErr> pose.tol1 && length(selectIdx) < pose.ks)
    
    %Pick best basis vector
    A = Msub*pose.BreVis;
    A(:,selectIdx) = NaN;
    lambdas = res'*A;
    [~,imax] = max(abs(lambdas));
    
    % Add basis to collection
    selectIdx = [selectIdx imax];
    pose.selectIdx = selectIdx;
    
    % Minimize reconstruction error and compute residual
    [camera, pose, res,res1,res2] = minimizeReconError(pose, camera,alphasq_high_e,param);
    Msub = kron(eye(length(pose.annoids),length(pose.annoids)),camera.M);
end

end



function pose = scaleInput(pose)
% Helper function to scale input so that K = eye(3)
pose.xyunscaled = pose.xy;
pose.xyh        = pose.K\pose.xy;
pose.xyvisible  = pose.xyh(1:2,pose.annoids);
end


function pose = reshapeBasis(pose)
% Helper function to reshape basis

Bnew=[];
for i =1:size(pose.BOMP,2);
    Bnew =cat(3,Bnew,reshape(pose.BOMP(:,i),pose.numPoints,3));
end
pose.Breshaped = Bnew;
pose.mur = reshape(pose.mu,pose.numPoints,3);
pose.Bfull = pose.BOMP;
[pose.Bre,pose.mure] = reArrangeBasis(pose.BOMP,pose.mu,pose.numPoints);

pose.BreTensor  = reshape(pose.Bre,3,pose.numPoints,[]);
pose.mureTensor = reshape(pose.mure,3,pose.numPoints,[]);
pose.mureVis    = pose.mureTensor(:,pose.annoids);
pose.mureVis    = pose.mureVis(:);
pose.BreVis     = pose.BreTensor(:,pose.annoids,:);
pose.BreVis     = reshape(pose.BreVis, 3*length(pose.annoids),[]);

end

function pose = getLimbLengths(pose)
% Helper function to compute limblengths.

connect         = skelConnectionMatrix(pose.skel);
[pose.I,pose.J] = ind2sub(size(connect), find(connect));
Xmu             = reshape(pose.mu,pose.numPoints,3);
Xmu             = [Xmu ones(length(pose.xy),1)];
pose.lengths    = squareform(pdist(Xmu));
pose.Xmu        = Xmu;
pose.lengths    = squareform(pdist(Xmu));

end


function [camera,pose,res,res1,res2] = minimizeReconError(pose,camera,alphasq_high_e,param)
selectIdx = pose.selectIdx;
optType = pose.optType;
mur = pose.mur;
I = pose.I;
J  = pose.J;
lengths = pose.lengths;
Bnew = pose.Breshaped;
xyvisible = pose.xyvisible;
B = pose.Bfull;
annoids = pose.annoids;
epsxy  = inf;
xyprev = zeros(2,pose.numPoints);
numIters = 0;
while(epsxy>pose.tol2 && numIters < pose.numIters2)
        Msub = kron(eye(length(annoids),length(annoids)),camera.M);
        A = Msub*pose.BreVis(:,selectIdx);
        b = xyvisible(:) - kron(ones(length(annoids),1),camera.taff) - kron(eye(length(annoids),length(annoids)),camera.M)*pose.mureVis;
        %% Equality constrained
        switch optType
            case 1
                C = [];
                d = [];
                alphasq = 0;
                for i = 1:length(I)
                    if(length(selectIdx)>1)
                        Bi = (squeeze(Bnew(I(i),:,selectIdx)));
                        Bj = (squeeze(Bnew(J(i),:,selectIdx)));
                    else
                        
                        Bi = (squeeze(Bnew(I(i),:,selectIdx)))';
                        Bj = (squeeze(Bnew(J(i),:,selectIdx)))';
                    end
                    mui = mur(I(i),:)';
                    muj = mur(J(i),:)';
                    C = [C; (Bi-Bj)] ;
                    d = [d; -(mui-muj)];
                    alphasq = alphasq + lengths(I(i),J(i))^2;
                end
                alpha = sqrt(alphasq); % alpha has the sum of limmb lengths 
                %                 alpha = sqrt(pose.lengthsum);
                [a,asolT] = leastsquareseqcon3(A,b,C,d,alpha);
                asol = asolT'; % coefficient to be multiplied by the basis (line158)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                n = length(selectIdx);
                CC = zeros( 3*length(pose.I) , 3*pose.numPoints);
                for i = 1:length(I)
                    CC(3*(i-1) + [1:3], 3*(pose.I(i)-1) + [1:3] ) = eye(3);
                    CC(3*(i-1) + [1:3], 3*(pose.J(i)-1) + [1:3] ) = -eye(3);                    
                end;
                BB = pose.BreVis(:,selectIdx);
                MU = pose.mureVis;
                alphasq_low  = alphasq - 1e-3;
                alphasq_high = alphasq + alphasq_high_e;
                P1 = (CC*BB)'*(CC*BB);
                q1 = -2*(CC*MU)'*(CC*BB);
                r1 = (CC*MU)'*(CC*MU)-alphasq_high;                
                cvx_begin
                    variable omega(n)
                    dual variables lam1
                    minimize norm(b - A*omega,2) 
                    lam1: quad_form(omega,P1) + q1*omega + r1 <= 0;                    
                cvx_end
                fprintf(2,'constraint = %f - %f\n', norm(CC*BB*omega - CC*MU) , alpha );
                res1=norm(b - A*asol);
                res2=norm(b - A*omega);
                fprintf(2,'cost function= %f (original code) %f (cvx) \n',norm(b - A*asol) ,norm(b - A*omega));
                %asol
                %omega
                %pause;
                asol = omega;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        Xnew = B(:,selectIdx)*asol + pose.mu;
        XnewR= reshape(Xnew,pose.numPoints,3);
        pose.Xnew = Xnew;
        pose.XnewR = XnewR;
        pose.XnewRt = XnewR';
        pose.Xnewre = pose.XnewRt(:);
        pose.XnewreVis = pose.XnewRt(:,annoids);
        pose.XnewreVis = pose.XnewreVis(:);
        
        XMissing=XnewR(annoids,:);
        xyMissing = xyvisible;
        pose.asol = asol;
        pose.selectIdx = selectIdx;
        
        xyrep1 = projectIntoAffineCam(XnewR,pose.K,camera.R,camera.t,camera.S,pose.skel);
        repErr1 = sum(sum((xyrep1(1:2,annoids)-pose.xyunscaled(1:2,annoids)).^2,1));
        
        %Estimate camera
        [cameraHyp]= estimateWPCamera(xyMissing',XMissing,param);
        
        if (0)
            %instead of estimating camera, fix the camera position
            %load('new_camera7'); % new_camera7 is wrong because new_camra.Raff
            %is different from [1 0 0;0 1 0]*R
            load('new_camera6');        
            %cameraHyp=new_camera;
            cameraHyp.R = new_camera.R;
            cameraHyp.M = new_camera.S*[1 0 0;0 1 0]*new_camera.R;
            cameraHyp.Raff = [1 0 0;0 1 0]*new_camera.R;
        end;
            
        %Compute reprojection error
        xyrep2 = projectIntoAffineCam(XnewR,pose.K,cameraHyp.R,cameraHyp.t,cameraHyp.S,pose.skel);
        repErr2 = sum(sum((xyrep2(1:2,annoids)-pose.xyunscaled(1:2,annoids)).^2,1));
        
        
        camera =cameraHyp;
        xyrep = xyrep2;
        repErr = repErr2;

        epsxy = sum(sum((xyrep(1:2,annoids)-xyprev(1:2,annoids)).^2,1));
        xyprev = xyrep;
        pose.repErr = repErr;
        
        % Visualize
        if(pose.viz)
            figure(4);clf;
            pointsVisualize(XnewR,pose.skel,'texton',0);hold on;
            drawCam(cameraHyp.R,cameraHyp.t,'gt',1);
     
            figure(3);clf;
            imshow(pose.im);hold on;
            plot2Dskeleton(xyrep',pose.skel,1,'texton',0);
            hold on;
            plot2Dskeleton(pose.xyunscaled',pose.skel,1,'texton',0,'gt',1);
            axis ij;
        end
        
        numIters = numIters + 1;
        pose.numIters = pose.numIters +1;
end

res = xyvisible(:) - kron(ones(length(annoids),1),camera.taff) - kron(eye(length(annoids),length(annoids)),camera.M)*pose.XnewreVis;
res = res- ((kron(eye(length(annoids),length(annoids)),camera.M))*B(:,selectIdx))*((kron(eye(length(annoids),length(annoids)),camera.M))*B(:,selectIdx))'*res;
end