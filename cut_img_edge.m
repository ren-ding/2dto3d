
function [texture]= cut_img_edge(texture)
    [p,q,~]=size(texture);
    
    left_idx=1;
    right_idx=1;
    for i=1:q
        if( sum(texture(:,i,1)) ~= 0) 
            left_idx = i;
            break;
        end
    end
    
    for i=q:-1:1
        if( sum(texture(:,i,1)) ~= 0) 
            right_idx = i;
            break;
        end
    end
    
    texture= texture(:,left_idx:right_idx,:);
end
