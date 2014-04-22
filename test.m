function [ output_args ] = test()
%TEST Summary of this function goes here
%   Detailed explanation goes here

load('test_tm2.mat');

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
        [p1,p2,p3,p4]= find_project_edge(pbx,pby);
        
        pbx_org=xy_mtx(1,kk);
        pby_org=xy_mtx(2,kk);
        org = [pbx_org;pby_org]';
        
        %only use p1 which are top circle projection edge
        [q,~]= size(org);
        shift_idx = 1;
        for j=1:q
            if (org(j,:)==p1)
                shift_res(shift_res_idx)=j;
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


disp(shift_res);

[gaussianMan,xy_mtx,idx_xy] = modelwithTexture(im,pose.K,camera.R,camera.t,camera.S,edges,pose.XnewR,pose.skel,adjust_res_five,adjust_tb_five);

end

