function [fid] = save3Dpose(vals,skel)
% save an xml file to be parsed by Adrian
%
%

joint_names = {'pelvis','left_hip','left_knee','left_foot',...
               'right_hip','right_knee','right_foot',...
               'clavicle','head','left_shoulder','left_elbow','left_hand',...
               'right_shoulder','right_elbow','right_hand'};
bone_names = {'left_pelvis','left_femur','left_calf',...
              'right_pelvis','right_femur','right_calf',...
              'torso','neck','left_clavicle','left_humerus','left_ulna',...
              'right_clavicle','right_humerus','right_ulna'};



connect = skelConnectionMatrix(skel);
indices = find(connect);
[I, J] = ind2sub(size(connect), indices);

figure(21);clf;
plot3(vals(:,1),vals(:,2),vals(:,3),'o');
for i = 1: size(vals,1)
    h1 = text( vals(i, 1), vals(i, 2), vals(i, 3),joint_names{i},'FontSize',10,'FontWeight','bold');
end
hold on;
for i=1:length(I)
    line([vals(I(i), 1) vals(J(i), 1)],...
         [vals(I(i), 2) vals(J(i), 2)], ...
         [vals(I(i), 3) vals(J(i), 3)]);
     h1 = text( (vals(I(i), 1)+vals(J(i), 1))/2, (vals(I(i), 2)+vals(J(i), 2))/2, ...
         (vals(I(i), 3)+vals(J(i), 3))/2,bone_names{i},'FontSize',10,'FontWeight','bold');
     radius(i) = 3;
end;
grid on;
axis ij;



fid = fopen('pose3D_.txt','w');

if (fid > 0)
    % counter joints
    fprintf(fid,'joints:%d\n',size(vals,1));
    for i=1:size(vals,1)
        fprintf(fid,'%s:%3.3f:%3.3f:%3.3f\n',joint_names{i},vals(i,1),vals(i,2),vals(i,3));
    end;
    fprintf(fid,'bones:%d\n',length(I));
    for i=1:length(I)
        fprintf(fid,'%s:%s:%s:%d\n',bone_names{i},joint_names{I(i)},joint_names{J(i)},radius(i));
    end;
    
    
    fclose(fid);
end;

