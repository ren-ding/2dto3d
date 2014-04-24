
function [new_texture]= img_rotation(j1,j2,pbx,pby,minX, minY,texture)
%     %vertical vector
%     v1 = [0,-1];
%     %body part's direction
%     direction_vec=p1-p2;
%     
%     a=acosd(dot(v1,direction_vec)/(norm(v1)*norm(direction_vec)));
%     
%     
%     
%     I = mat2gray(texture);
%     texture = imrotate(I,a,'bilinear','crop');
%     
%     [texture]= cut_img_edge(texture);
 %finding the texture linear transform from original to adjust
        %texture
        %p2------p3            xt2-----xt3
        %|        |            |        |
        % |        | --------> |        |
        %  |        |          |        |
        %   p1------p4         xt1-----xt4
        
        [p1,p2,p3,p4]= find_project_edge_ordered(j1,j2,pbx,pby);
        p1=p1-[minX minY];
        p2=p2-[minX minY];
        p3=p3-[minX minY];
        p4=p4-[minX minY];
        
        
        xt1=[0,100];
        xt2=[0,0];
        xt3=[100,0];
        xt4=[100,100];

        xt = [xt1',xt2',xt3',xt4';ones(1,4)];
        %p=[p1',p2',p3',p4';ones(1,4)];

        
        num_p=4;
        A = zeros(num_p*2,6);
        
        for i=1:num_p
            A(2*(i-1)+1,1:3) = xt(:,i)';%p(:,i); 
            A(2*(i-1)+2,4:6) = xt(:,i)';%p(:,i);  
        end;
        
        %B=[xt1';xt2';xt3';xt4'];
        B=[p1';p2';p3';p4'];
        
        M=pinv(A)*B;
        
        T=[M(1),M(2),M(3);M(4),M(5),M(6);0,0,1];
           
        I = mat2gray(texture);
        
        
        
        new_texture = ones(100,100,3);
        mask_inputimage = poly2mask(pbx-minX,pby-minY,size(texture,1),size(texture,2));
        
        %[w,h,~]=size(texture);
        [w,h,~]=size(new_texture);
        
        for i=1:h
            for j=1:w
                newP = T*[i;j;1];
                newP = int32(newP);
                if(newP(1)>0 && newP(1)< size(texture,2) && newP(2)>0&& newP(2)<size(texture,1))
                    if(mask_inputimage(newP(2),newP(1))==1)
                        new_texture(j,i,:) = I(newP(2),newP(1),:);
                    end;
                end
            end
        end
        %imshow(new_texture);
%         movingPoints = [xt1;xt2;xt3;xt4];
%         fixedPoints  = [p1;p2;p3;p4];
%         t_piecewise_linear = fitgeotrans(movingPoints,fixedPoints,'pwl');
%         I_piecewise_linear = imwarp(I,t_piecewise_linear);

        
        
end
