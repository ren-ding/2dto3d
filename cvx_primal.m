
% load data
load ('poseandcamera');

% define variables


% train svm using cvx
cvx_begin
    variables asol
    minimize 
    A*asol-Msub*mu == alphasq;
cvx_end

% visualize
