%flip 3d array up and down
function [arr]= flipud3D(arr)
    arr(:,:,1)=flipud(arr(:,:,1));
    arr(:,:,2)=flipud(arr(:,:,2));
    arr(:,:,3)=flipud(arr(:,:,3));
end
