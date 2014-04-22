function [ obj ] = test1()
 
addpath mesh;
load('test_tm2.mat');
 
% obj = read_wobj('mesh1.obj');
% vertices = obj.vertices;
% [vertices_size,~] = size(vertices);
% verticesh = [vertices'; ones(1,vertices_size) ];
 
%transform points from havok to matlab
 
%To space co-ordinates
xh=[X';ones(1,15)];
x=X';


%From space co-ords
yh = zeros(4,15);

fid = fopen('outputIK1.pse');

tline = fgets(fid);
while ischar(tline)
    C = strsplit(tline);
    [~,cs] = size(C);
    for i=1:cs
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mHipLeft'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mKneeLeft'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mAnkleLeft'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mHipRight'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mKneeRight'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mAnkleRight'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
        if(strcmp(C{i},'mPelvis'))
            disp( [C{i+1},' ' , C{i+2},' ',C{i+3},' ',C{i+4}]);
        end
    end
    
    tline = fgets(fid);
end

fclose(fid);

yh=[[5.90332e-009 1.135 0 1.0]',[0.128946 1.0815 -6.06403e-005 1.0]',[0.102461 0.648361 -0.0772218 1.0]',...
    [0.0679195 0.174396 -0.21833 1.0]',[-0.139142 1.08 -5.28991e-007 1.0]',[-0.208123 0.688723 0.190792 1.0]',...
    [-0.294744 0.212031 0.295717 1.0]',[-5.35627e-007 1.5 0 1.0]',[-6.72346e-007 1.69468 -0.0109919 1.0]',...
    [0.195236 1.63493 -0.0349813 1.0]',[0.286885 1.42426 0.0798222 1.0]',[0.398921 1.26854 0.285247 1.0]',...
    [-0.19496 1.63426 -0.0350012 1.0]',[-0.255211 1.43012 0.109941 1.0]',[-0.298722 1.24793 0.320079 1.0]']; % homog. coords in matlab
y=yh(1:3,1:15);



% finding the linear transform from matlab  to havok
nmb_points = size(x,2);
 
A = zeros(45,12);
for i=1:nmb_points
    A(3*(i-1)+1,1:4) = yh(:,i); 
    A(3*(i-1)+2,5:8) = yh(:,i); 
    A(3*(i-1)+3,9:12) = yh(:,i); 
end;
 
m = pinv(A)*x(:);
M = [m(1:4)';m(5:8)';m(9:12)';[0 0 0 1]]; % 4x4 Transf. matrix 
 
% transform points from havok to matlab
%Minv = inv(M);

%S=eye(4); S(3,3)=1e-3;
%M = M*S;

y_tmp = M*verticesh; % y_tmp and yh must be very similar
 
y_tmp_i= y_tmp';
 
vertices_back=y_tmp_i(1:vertices_size,1:3);
%draw 3D model in matlab and compute the projection
figure(1);
handle(1) = plot3(vertices_back(:, 1), vertices_back(:, 2), vertices_back(:, 3), '.');
axis ij % make sure the left is on the left.
set(handle(1), 'markersize', 1);
hold on
grid on
view([0,-90]);
%view([-90,90]);

figure(2);
xy = projectBack([vertices_back(:, 1), vertices_back(:, 2), vertices_back(:, 3)],pose.K,camera.R,camera.t,camera.S);    

imshow(im);
hold on

%xy=flipud(xy);
%view([-90,90]);

plot(xy(1,:),xy(2,:),'or');
end
 

