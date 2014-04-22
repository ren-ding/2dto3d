
% # Description of the file format
% # <HavokJoint> position<x y z> : <Pose Detection Joint> position<x y z>
% 
% root 0 1.135 0 0 : <pelvis>:0.545:-0.657:-0.257
% 
% LeftHip 0.13 -0.0549995 -6.05952e-005 0 : <left_hip>:-1.215:-1.809:0.932
% 
% RightHip -0.13 -0.0549995 0 0 : <right_hip>:2.074:-2.231:0.638
% 
% spine3 -1.74983e-009 0.125009 0 0 : <clavicle>:0.390:5.468:-0.135

xh=[[0 1.135 0 1]',[0.13 -0.0549995 -6.05952e-005 1]',[-0.13 -0.0549995 0 1]',[-1.74983e-009 0.125009 0 1]']; % homog. coords in havok
x=[[0 1.135 0]',[0.13 -0.0549995 -6.05952e-005]',[-0.13 -0.0549995 0]',[-1.74983e-009 0.125009 0]'];
yh=[[0.545 -0.657 -0.257 1]',[-1.215 -1.809 0.932 1]',[2.074 -2.231 0.638 1]',[0.390 5.468 -0.135 1]']; % homog. coords in matlab
y=[[0.545 -0.657 -0.257]',[-1.215 -1.809 0.932]',[2.074 -2.231 0.638]',[0.390 5.468 -0.135]'];

nmb_points = size(x,2);

A = zeros(12,12);
for i=1:nmb_points
    A(3*(i-1)+1,1:4) = yh(:,i); 
    A(3*(i-1)+2,5:8) = yh(:,i); 
    A(3*(i-1)+3,9:12) = yh(:,i); 
end;

% finding the linear transform from matlab  to havok
m = pinv(A)*x(:);
M = [m(1:4)';m(5:8)';m(9:12)';[0 0 0 1]]; % 4x4 Transf. matrix 

% transform points from matlab to havok
x_tmp = M*yh; % x_tmp and xh must be very similar
disp(['error: ',num2str(norm(xh(:) - x_tmp(:)))]);

% transform points from havok to matlab
Minv = inv(M);
y_tmp = Minv*xh; % y_tmp and yh must be very similar
disp(['error: ',num2str(norm(norm(yh(:) - y_tmp(:))))]);






