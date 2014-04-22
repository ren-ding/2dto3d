function [gaussianMan,xy_mtx,idx_xy] = modelwithTexture(im,poseK,cameraR,camerat,cameraS,edges,vals,skel,params_mid_raduis,params_tb_raduis,varargin)
%mid raduis parameters: body,upper arms,low arms, uppper legs, low legs
%params_mid_raduis = [0,0,0,0,0];

%raduis parameters: body top/bottom,upper arms top/bottom,low arms top/bottom,
%uppper legs top/bottom, low legs top/bottom
%params_tb_raduis = zeros(1,10);

[color,type,person,connect]=process_options(varargin,'color','r','type',1,'person',1,'connect',[]);

gaussianMan=[];
xy_mtx=[];
idx_xy=[];idx_nmb=0;

for i =1:size(vals,1)
    strs{i}={int2str(i)};
end
if isempty(connect)
    connect = skelConnectionMatrix(skel);
end

indices = find(connect);
[I, J] = ind2sub(size(connect), indices);
handle(1) = plot3(vals(:, 1), vals(:, 3), vals(:, 2), '.');
axis ij % make sure the left is on the left.
set(handle(1), 'markersize', 1);
hold on
grid on


if(type<4)
    %edges need to be matched
    edges=edges/10;
    %head
    edges(1)=edges(1)/1.25;
    %torso
    edges(6)=edges(6)/2;

    %set the mean of first three minimal values as the limbs'thickness
    edge_arm=sort([edges(2),edges(3),edges(4),edges(5)]);
    edge_leg=sort([edges(7),edges(8),edges(9),edges(10)]);
    edge_arm=(edge_arm(1)+edge_arm(2)+edge_arm(3))/3;
    edge_leg=(edge_leg(1)+edge_leg(2)+edge_leg(3))/3;
    
    xy_mtx=[];
    P_bot_body=[0,0,0];
    %calculate the average of sphere1 and sphere14 which is the hip
    s1 = [0,0,0];
    
    for i = 1:length(indices)
        P1 = [vals(I(i),1) vals(I(i),3) vals(I(i),2)];
        P2 = [vals(J(i),1) vals(J(i),3) vals(J(i),2)];
        P = 0.5*(P1 + P2);
       
        if(i==8)
            %head
            tex= imread(['texture/12.jpg']);
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(8);
            r=edges(1);
            x=r*x+P2(1);y=r*y+P2(2);z=r*z+P2(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');

            %head projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            %body
            tex= imread(['texture/13.jpg']);
            P_top_body=(P+P1)/2;
            r=edges(6)+params_mid_raduis(1);
            [x,y,z]=cylinder2P([r+params_tb_raduis(1),r+params_tb_raduis(2)],8,P_top_body,P_bot_body);
          
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %body projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
        end    
        
        if(i==1)
            s1=P;
        end
        
        if(i==4)
            %hip
            tex= imread(['texture/6.jpg']);
            [tex]= flipud3D(tex);
            P = (P+s1)/2;
            [x,y,z]=sphere(4);
            r=edge_leg;
            x=r*x+P(1);y=r*y+P(2);z=r*z+P(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %hip projection            
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
        end
        
        if(i==9||i==12)
        end
        
        if(i==7)
            %get the bottom joint of body
            P_bot_body=P1;
        end
        
        if(i==10||i==13)
            %joints of body and arms
            if(i==10)
                tex= imread(['texture/14.jpg']);
            end
            if(i==13)
                tex= imread(['texture/19.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_arm+params_mid_raduis(2)+params_tb_raduis(3);
            x=r*x+P1(1);y=r*y+P1(2);z=r*z+P1(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %joints of body and arms projection            
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            %upper arms
            if(i==10)
                tex= imread(['texture/15.jpg']);
            end
            if(i==13)
                tex= imread(['texture/20.jpg']);
            end
            [x,y,z]=cylinder2P([edge_arm+params_mid_raduis(2)+params_tb_raduis(3),edge_arm+params_mid_raduis(2)+params_tb_raduis(4)],8,P1,P2);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %upper arms projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))]; 
        end
        
        if(i==11||i==14)
            %joints of upper and lower arms
            if(i==11)
                tex= imread(['texture/16.jpg']);
            end
            if(i==14)
                tex= imread(['texture/21.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_arm+params_mid_raduis(3)+params_tb_raduis(5);
            x=r*x+P1(1);y=r*y+P1(2);z=r*z+P1(3); 
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %joints of upper and lower arms projection           
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            %hands
            if(i==11)
                tex= imread(['texture/17.jpg']);
            end
            if(i==14)
                tex= imread(['texture/22.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_arm+params_mid_raduis(3)+params_tb_raduis(6);
            x=r*x+P2(1);y=r*y+P2(2);z=r*z+P2(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %hands projection            
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            %lower arms
            if(i==11)
                tex= imread(['texture/18.jpg']);
            end
            if(i==14)
                tex= imread(['texture/23.jpg']);
            end
            [x,y,z]=cylinder2P([edge_arm+params_mid_raduis(3)+params_tb_raduis(5),edge_arm+params_mid_raduis(3)+params_tb_raduis(6)],8,P1,P2);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %lower arms projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
        end 
        
        if(i==2||i==5)
            %joints of body and legs
            if(i==2)
                tex= imread(['texture/1.jpg']);
            end
            if(i==5)
                tex= imread(['texture/7.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_leg+params_mid_raduis(4)+params_tb_raduis(7);
            x=r*x+P1(1);y=r*y+P1(2);z=r*z+P1(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %joints of body and legs projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            
            
            %upper legs
            if(i==2)
                tex= imread(['texture/2.jpg']);
            end
            if(i==5)
                tex= imread(['texture/8.jpg']);
            end
            [x,y,z]=cylinder2P([edge_leg+params_mid_raduis(4)+params_tb_raduis(7),edge_leg+params_mid_raduis(4)+params_tb_raduis(8)],8,P1,P2);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %upper legs projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
        end   
        if(i==3||i==6) 
            %joints of upper and lower legs
            if(i==3)
                tex= imread(['texture/3.jpg']);
            end
            if(i==6)
                tex= imread(['texture/9.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_leg+params_mid_raduis(5)+params_tb_raduis(9);
            x=r*x+P1(1);y=r*y+P1(2);z=r*z+P1(3);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %joints of upper and lower legs            
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
            
            %feet
            if(i==3)
                tex= imread(['texture/4.jpg']);
            end
            if(i==6)
                tex= imread(['texture/10.jpg']);
            end
            [tex]= flipud3D(tex);
            [x,y,z]=sphere(4);
            r=edge_leg+params_mid_raduis(5)+params_tb_raduis(10);
            x=r*x+P2(1);y=r*y+P2(2);z=r*z+P2(3); 
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %feet projection           
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
                        
            %lower legs
            if(i==3)
                tex= imread(['texture/5.jpg']);
            end
            if(i==6)
                tex= imread(['texture/11.jpg']);
            end
            [x,y,z]=cylinder2P([edge_leg+params_mid_raduis(5)+params_tb_raduis(9),edge_leg+params_mid_raduis(5)+params_tb_raduis(10)],8,P1,P2);
            surf(x,y,z,tex, 'edgecolor', 'none','FaceColor','texturemap');
            
            %lower legs projection
            xy = projectBack([x(:),z(:),y(:)],poseK,cameraR,camerat,cameraS);
            xy_mtx=[xy_mtx,xy];
            idx_nmb=idx_nmb+1;idx_xy=[idx_xy,repmat(idx_nmb,1,size(xy,2))];
        end
       
    end
    axis equal
    
else
    
end

axis on
hold off
