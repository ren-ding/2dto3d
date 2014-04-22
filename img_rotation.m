
function [texture]= img_rotation(p1,p2,texture)
    %vertical vector
    v1 = [0,-1];
    %body part's direction
    direction_vec=p1-p2;
    
    a=acosd(dot(v1,direction_vec)/(norm(v1)*norm(direction_vec)));
    
    if (a>45)
        a = a-90;
    end
    
    I = mat2gray(texture);
    texture = imrotate(I,a,'bilinear','crop');
    
    [texture]= cut_img_edge(texture);
end
