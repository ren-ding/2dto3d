function [texture]= get_texture(pbx,pby,im,idx,xy,shift)
    
    %attention, figure x,y are opesite
    [m,n,k]=size(im);
    BW = poly2mask(pbx, pby, m, n);
    
    BW3=cat(3,BW,BW,BW);
    texture = uint8(double(BW3)).*im;
    
    minX=min(pbx);
    maxX=max(pbx);
    minY=min(pby);
    maxY=max(pby);
    if(minX <0)
        minX=1;
    end
    if(minY<0)
        minY=1;
    end
    
    imshow(texture);
    %pause;
   
    %disp([minY,maxY,minX,maxX]);
    %f=figure(idx);
    texture=texture(minY:maxY,minX:maxX,:);
    
    %rotation to adjust image

    if(shift > 7)
        shift=shift-8;
    end
    if(idx==2)
        %right upper leg
        texture = img_rotation(xy(:,2)',xy(:,3)',texture);
        texture=align_texture(texture,shift);
    elseif  (idx==5)
        %right lower leg
        texture = img_rotation(xy(:,3)',xy(:,4)',texture);
        texture=align_texture(texture,shift);
    elseif  (idx==8)
        %left upper leg
        texture = img_rotation(xy(:,5)',xy(:,6)',texture);
        texture=align_texture(texture,shift);
    elseif  (idx==11)
        %left lower leg
        texture = img_rotation(xy(:,6)',xy(:,7)',texture);
        texture=align_texture(texture,shift);
    elseif  (idx==12)
        %head
        texture=align_texture(texture,shift);
    elseif  (idx==13)
        %body
        texture = img_rotation(xy(:,8)',xy(:,1)',texture);
        texture=align_texture(texture,shift);
    elseif (idx==15)
        %right upper arm
        texture = img_rotation(xy(:,10)',xy(:,11)',texture);
        texture=align_texture(texture,shift);
    elseif (idx==18)
        %right lower arm
        texture = img_rotation(xy(:,11)',xy(:,12)',texture);
        texture=align_texture(texture,shift);
    elseif (idx==20)
        %left upper arm
        texture = img_rotation(xy(:,13)',xy(:,14)',texture);
        texture=align_texture(texture,shift);
    elseif (idx==23)
        %left lower arm
        texture = img_rotation(xy(:,14)',xy(:,15)',texture);
        texture=align_texture(texture,shift);
    else  
        %second param:shift param's range should from -7 to 7
        texture=align_texture(texture,shift);
    end
    
    imwrite(texture,['texture/', num2str(idx),'.jpg']);
end
