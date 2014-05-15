function [BW2,rs]=edge_detect(name)
    %results:head,left arm upper, left arm lower
    %right arm upper, right arm lower
    %torso,
    %left leg upper, left leg lower
    %right leg upper, right leg lower
    rs=zeros(1,10);
    load(name);
    gray_im= rgb2gray(im);
    BW2 = edge(gray_im,'canny');
    figure, imshow(BW2);

    [imsizey,imsizex] = size(BW2);
    points=xy';

    %head part
    rs(1)=edge_eachpart(points(8,:), points(9,:),points(9,:),BW2,50);
    
    %right arm
    %upper right arm
    upper_rarm_center=(points(14,:)+points(13,:))/2;
    rs(2)=edge_eachpart(points(14,:), points(13,:),upper_rarm_center,BW2,50);
    %lower right arm
    lower_rarm_center=(points(15,:)+points(14,:))/2;
    rs(3)=edge_eachpart(points(15,:), points(14,:),lower_rarm_center,BW2,50);

    %left arm
    %upper left arm
    upper_larm_center=(points(11,:)+points(10,:))/2;
    rs(4)=edge_eachpart(points(11,:), points(10,:),upper_larm_center,BW2,50);
    %lower left arm
    lower_larm_center=(points(12,:)+points(11,:))/2;
    rs(5)=edge_eachpart(points(12,:), points(11,:),lower_larm_center,BW2,50);

    %torso
    torsocenter_point=(points(1,:)+points(8,:))/2;
    rs(6)=edge_eachpart(points(1,:), points(8,:),torsocenter_point,BW2,90);

    %right leg
    %upper right leg
    upper_rleg_center=(points(6,:)+points(5,:))/2;
    rs(7)=edge_eachpart(points(6,:), points(5,:),upper_rleg_center,BW2,50);
    %lower right leg
    lower_rleg_center=(points(7,:)+points(6,:))/2;
    rs(8)=edge_eachpart(points(7,:), points(6,:),lower_rleg_center,BW2,50);

    %left leg
    %upper left leg
    upper_lleg_center=(points(3,:)+points(2,:))/2;
    rs(9)=edge_eachpart(points(3,:), points(2,:),upper_lleg_center,BW2,50);
    %lower left leg
    lower_lleg_center=(points(4,:)+points(3,:))/2;
    rs(10)=edge_eachpart(points(4,:), points(3,:),lower_lleg_center,BW2,50);
end

function [min_units] = edge_eachpart(point1, point2,center_point,BW2,looptimes)
    rotate_matrix=[cos(-pi/2),sin(-pi/2);-sin(-pi/2),cos(-pi/2)];
    vt_vector=point1-point2;
    ht_vector=(rotate_matrix*vt_vector')';
    ht_norm=ht_vector/norm(ht_vector);

    left_units=0;
    right_units=0;
    for units=1:looptimes
        edge_point1=center_point-ht_norm*units;
        edge_point2=center_point+ht_norm*units;

        %display the processing
        %hold on;
        %plot width line
        %plot(edge_point1(1),edge_point1(2),'.','MarkerEdgeColor','g','linewidth',1);
        %plot(edge_point2(1),edge_point2(2),'.','MarkerEdgeColor','g','linewidth',1);
        %pause(.1);
        if(edge_point1(1) < 1 || edge_point1(2) < 1 || edge_point2(1) < 1 || edge_point2(2) < 1)
            break;
        end
        
        bw2_edge_flag1=BW2(int32(edge_point1(2)),int32(edge_point1(1)));
        bw2_edge_flag2=BW2(int32(edge_point2(2)),int32(edge_point2(1)));
        if(bw2_edge_flag1==1)
            left_units=units;
        end
        if(bw2_edge_flag2==1)
            right_units=units;
        end
        %disp([num2str(bw2_edge_flag1),',',num2str(bw2_edge_flag2)]);
    end
    %pick the smaller edge between left and right
    min_units=min(left_units,right_units);
    
    %disp(num2str(min_units));
end