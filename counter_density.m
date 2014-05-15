%pbx,pby:polygon edge points
%canny_image:grey image with canny edge detection
function [density]= counter_density(pbx,pby,canny_image)
    %attention, figure x,y are opesite
    [m,n]=size(canny_image);
    BW = poly2mask(pbx, pby, m, n);
    imBW=BW&canny_image;
    
    %calculate the density
    density = sum( imBW(:))/sum(BW(:));
end
