function [ obj ] = test1()
 
addpath mesh;
load('test_tm2.mat');
 
% obj = read_obj('mesh1.obj');
vertices = obj.vertices;

[vertices_size,~] = size(vertices);
verticesh = [vertices'; ones(1,vertices_size) ];

%transform points from havok to matlab

%To space co-ordinates
xh=[X';ones(1,15)];
x=X';

%From space co-ords
% homog. coords in matlab
yh = zeros(4,15);

fid = fopen('outputIK1.pse');
tline = fgets(fid);
while ischar(tline)
    C = strsplit(tline);
    [~,cs] = size(C);
    for i=1:cs
        if(strcmp(C{i},'mPelvis'))
            yh(:,1)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mHipLeft'))
            yh(:,2)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mKneeLeft'))
            yh(:,3)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mAnkleLeft'))
            yh(:,4)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mHipRight'))
            yh(:,5)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mKneeRight'))
            yh(:,6)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mAnkleRight'))
            yh(:,7)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mChest'))
            yh(:,8)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mHead'))
            yh(:,9)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mShoulderLeft'))
            yh(:,10)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mElbowLeft'))
            yh(:,11)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mWristLeft'))
            yh(:,12)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mShoulderRight'))
            yh(:,13)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mElbowRight'))
            yh(:,14)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
        if(strcmp(C{i},'mWristRight'))
            yh(:,15)=[str2num(C{i+1}), str2num(C{i+2}),str2num(C{i+3}),str2num(C{i+4})];
        end
    end
    
    tline = fgets(fid);
end

fclose(fid);
y=yh(1:3,1:15);

% finding the linear transform from matlab to havok
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


%vertices_back = vertices_back+repmat([1,0,0],vertices_size,1);

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

%
%density computing processing...
%
%the number of sub windows is plit_pieces*plit_pieces
plit_pieces=12;

grey_img= rgb2gray(im);
canny_image = edge(grey_img,'canny');
[m,n]=size(canny_image);

%calculate the moving window steps
minX=fix(min(xy(1,:))-1);
maxX=ceil(max(xy(1,:)));
minY=fix(min(xy(2,:))-1);
maxY=ceil(max(xy(2,:)));
stepX = ceil((maxX-minX)/plit_pieces);
stepY = ceil((maxY-minY)/plit_pieces);

%make sure the projection point are positive
if(minX < 1 && minX >=0)
    xy(1,:) = xy(1,:)+ 1;
    minX=minX+1;
    maxX=maxX+1;
end
if(minY < 1 && minY >=0)
    xy(2,:) = xy(2,:)+ 1;
    minY=minY+1;
    maxY=maxY+1;
end
if(minX < 0)
    xy(1,:) = xy(1,:)+ abs(minX)+1;
    minX=1;
    maxX=maxX+abs(minX)+1;
end
if(minY < 0)
    xy(2,:) = xy(2,:)+ abs(minY)+1;
    minY=1;
    maxY=maxY+abs(minX)+1;
end


convhull_xy =[];
final_BW=poly2mask(0, 0, m, n);

%here stepX/2,stepY/2 which means overlaping, remove the space between each
%window
for i=minX:stepX/2:maxX
    for j=minY:stepY/2:maxY
        ii=i+stepX;
        jj=j+stepY;
        if(ii > maxX)
            ii=maxX;
        end
        if(jj>maxY)
            jj=maxY;
        end
        
        %get all sub points from each window
        sub_xy=[];
        sub_idx=0;
        [~,s]=size(xy);
        for k=1:s
            if(xy(1,k)>=i && xy(1,k)<=ii && xy(2,k)>=j && xy(2,k)<=jj)
                sub_idx=sub_idx+1;
                sub_xy(:,sub_idx)=xy(:,k);
            end
        end
        
        %calculate the sub convex hull
        if(sub_idx>2 )
            ih = convhull(sub_xy(1,:),sub_xy(2,:));
            convhull_xy=[convhull_xy, sub_xy(:,ih)];
            sub_BW = poly2mask(sub_xy(1,ih), sub_xy(2,ih), m, n);
            final_BW=final_BW|sub_BW;
        end
%         disp([num2str(i),' ',num2str(j)]);
%         disp([num2str(ii),' ',num2str(jj)]);
    end
end


%xy=flipud(xy);
%view([-90,90]);

plot(xy(1,:),xy(2,:),'.r');
plot(convhull_xy(1,:),convhull_xy(2,:),'.b');
mask_im=uint8(final_BW)*255;
figure
imshow(mask_im);

%calculate the density
imBW=final_BW&canny_image;
%calculate the density
density = sum( imBW(:))/sum(final_BW(:));

disp(['density:' num2str(density)]);

end
