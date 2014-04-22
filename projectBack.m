function xy = projectBack( X, K,R,t,S,varargin )
%PROJECTBACK Summary of this function goes here
%   Detailed explanation goes here

[viz,s]=process_options(varargin,'viz',0,'scale',1);

%% Do projection
numPts =size(X,1);

%Create Affine camera
Raff =[ R(1,:); R(2,:); zeros(1,3)];
taff =K*[ t(1); t(2); 1]; % TODO: z-dim should be scaling
M=K*Raff;


%Project into affine camera
xy = S*M(1:2,:)*X' + repmat(taff(1:2),[1,numPts]);
xy(3,:) = ones(1,size(xy,2));

xy=xy(1:2,:);
end

