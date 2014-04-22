%pbx,pby:polygon edge points
%grey_image:grey image
function [density]= counter_density(pbx,pby,grey_img)
    %attention, figure x,y are opesite
    [m,n]=size(grey_img);
    BW = poly2mask(pbx, pby, m, n);
    imBW=BW&grey_img;
    
    %calculate the density,sum(BW(:))/50000 
    density = sum( imBW(:))/sum(BW(:));
end
