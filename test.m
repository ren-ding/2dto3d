function [ output_args ] = test()
%TEST Summary of this function goes here
%   Detailed explanation goes here

load('tmp1.mat');

adjust_xy=projectIntoAffineCam(Xnew1,pose.K,camera.R,camera.t,camera.S,pose.skel);
adjust_xy=[adjust_xy(1,:);adjust_xy(2,:)];

shift_res = ones(1,9);
shift_res_idx = 1;

for i = all_idx
    kk = find(idx_xy == i);
    ih = convhull(xy_mtx(1,kk),xy_mtx(2,kk));
    pbx=xy_mtx(1,kk(ih));
    pby=xy_mtx(2,kk(ih));

    if(i==2 ||i==5 ||i==8 ||i==11 ||i==13 ||i==15 ||i==18 ||i==20 ||i==23)
        
        if(i==2)
            %right upper leg
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,2)',adjust_xy(:,3)',pbx,pby);
        elseif  (i==5)
            %right lower leg
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,3)',adjust_xy(:,4)',pbx,pby);
        elseif  (i==8)
            %left upper leg
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,5)',adjust_xy(:,6)',pbx,pby);
        elseif  (i==11)
            %left lower leg
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,6)',adjust_xy(:,7)',pbx,pby);
        elseif  (i==13)
            %body
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,8)',adjust_xy(:,1)',pbx,pby);
        elseif (i==15)
            %right upper arm
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,10)',adjust_xy(:,11)',pbx,pby);
        elseif (i==18)
            %right lower arm
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,11)',adjust_xy(:,12)',pbx,pby);
        elseif (i==20)
            %left upper arm
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,13)',adjust_xy(:,14)',pbx,pby);
        elseif (i==23)
            %left lower arm
            [p1,p2,p3,p4]= find_project_edge_ordered(adjust_xy(:,14)',adjust_xy(:,15)',pbx,pby);
        end
        
        %[p1,p2,p3,p4]= find_project_edge(pbx,pby);
        
        pbx_org=xy_mtx(1,kk);
        pby_org=xy_mtx(2,kk);
        org = [pbx_org;pby_org]';
        
        %only use p1 which are top circle projection edge
        [q,~]= size(org);
        shift_idx = 1;
        for j=1:q
            if (org(j,:)==p1)
                if(j>7)
                    shift_res(shift_res_idx)=j-8;
                else
                    shift_res(shift_res_idx)=-j;
                end
                
                break;
            end
        end
        get_texture(pbx,pby,im,i,adjust_xy,shift_res(shift_res_idx));
        
        shift_res_idx = shift_res_idx+1;
    else
        %do not shift
        get_texture(pbx,pby,im,i,adjust_xy,0);
    end
    
end


%disp(shift_res);

[gaussianMan,xy_mtx,idx_xy] = modelwithTexture(R2_idx,im,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,adjust_res_five,adjust_tb_five);

end

