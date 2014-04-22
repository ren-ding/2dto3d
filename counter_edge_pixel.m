%Linear interpolation
%counter how many white pixels hit the line p1->p2 
function [counter]= counter_edge_pixel(p1,p2,f)
    %attention, figure x,y are opesite
    distMatrix = [p1(1),p1(2);p2(1),p2(2)];
    dist = pdist(distMatrix,'euclidean');
    
    %Linear interpolation
    unit = (p2-p1)/dist;%unit vector
    points = zeros(fix(dist)+2,2);
    points(1,:) = p1;
    points(fix(dist)+2,:) = p2;
    
    %start to count image covering pixel
    counter=0;
    for i=1:fix(dist)
        points(i+1,:)=p1+i*unit;
        
        if(f(  round( points(i+1,2)),round(points(i+1,1)) ) ~=0 )
            counter = counter+1;
        end
    end
    
end
