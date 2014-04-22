function xy = showboxes(im, boxes, partcolor,filename)
% xy = showboxes(im, boxes, partcolor,filename)
% Detect joints of each body part, there are 15 joints in total.
imagesc(im); axis image; axis off;
if ~isempty(boxes)
  numparts = length(partcolor);
  box = boxes(:,1:4*numparts);
  xy = reshape(box,size(box,1),4,numparts);
  xy = permute(xy,[1 3 2]);
	x1 = xy(:,:,1);
	y1 = xy(:,:,2);
	x2 = xy(:,:,3);
	y2 = xy(:,:,4);
    
    [pathstr, name, ext]=fileparts(filename);
    
    im_handler=imshow(im);
    [imsizex,imsizey] = size(im);
    
    old_points = zeros(size(xy,2),2);
    new_points = zeros(15,2);
    
	for p = 1:size(xy,2)
        avg_x=(x1(:,p)+x2(:,p))/2; 
        avg_y=(y1(:,p)+y2(:,p))/2;
        old_points(p,1)=avg_x;
        old_points(p,2)=avg_y;
        
        %display joints
        %hold on;
        %plot(avg_x,avg_y,'o','MarkerEdgeColor',partcolor{p},'linewidth',2);
    end
    
    new_points(1,:)=(old_points(9,:)+old_points(10,:)+old_points(21,:)+old_points(22,:))/4;
    new_points(2,:)=old_points(22,:);
    new_points(3,:)=old_points(24,:);
    new_points(4,:)=old_points(26,:);
    new_points(5,:)=old_points(10,:);
    new_points(6,:)=old_points(12,:);
    new_points(7,:)=old_points(14,:);
    %new_points(8,:)=old_points(2,:);
    %new_points(8,:)=(old_points(3,:)+old_points(8,:)+old_points(15,:)+old_points(20,:))/4;
    new_points(8,:)=(old_points(2,:)+old_points(3,:)+old_points(8,:)+old_points(15,:)+old_points(20,:))/5;
    new_points(9,:)=old_points(1,:);
    new_points(10,:)=old_points(15,:);
    new_points(11,:)=old_points(17,:);
    new_points(12,:)=old_points(19,:);
    new_points(13,:)=old_points(3,:);
    new_points(14,:)=old_points(5,:);
    new_points(15,:)=old_points(7,:);
end
xy=new_points';
%drawnow
saveas(im_handler,['results/',name,'_done'],'jpg');