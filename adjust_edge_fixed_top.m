%shift raduis to adjust edge by most hitted pixel
%raduis_results: how many 'unit vector' need to be shifted

%body_part:
%1:body,2:upper arms,3:low arms,4: uppper legs,5: low legs
%body_part_ext:
%13:body,15,20:upper arms,18,23:low arms,
%2,8: uppper legs,5,11: low legs
function [raduis_results,tb_results]= adjust_edge_fixed_top(xy_mtx,idx_xy,poseK,cameraR,camerat,cameraS,edges,poseXnewR,poseskel,grey_img,body_part,body_part_ext,top_param)
    %res:counter how many sample points touch edges
    
    size_j=1;
    size_k=9;
    res=zeros(1,11*size_j*size_k);
    
    res_index=1;
    params_mid_raduis = zeros(1,5);
    params_tb_raduis = zeros(1,10);
    
    %mid raduis
    from_i=-0.5;
    to_i=-from_i;
    step_i=to_i/5;
    
    %top circle raduis = top_param
    
    %bottom circle raduis
    from_k=-0.4;
    to_k=-from_k;
    step_k=to_k/4;
    
    for i=from_i:step_i:to_i
        params_mid_raduis(body_part)=i;
        j=top_param;
            params_tb_raduis(body_part*2-1)=j;
            for k=from_k:step_k:to_k
                params_tb_raduis(body_part*2)=k;
                
                [~,xy_mtx,idx_xy] = visualizeGaussianModel(poseK,cameraR,camerat,cameraS,edges,poseXnewR,poseskel,params_mid_raduis,params_tb_raduis);

                kk = find(idx_xy == body_part_ext);
                ih = convhull(xy_mtx(1,kk),xy_mtx(2,kk));
                pbx=xy_mtx(1,kk(ih));
                pby=xy_mtx(2,kk(ih));

                %[p1,p2,p3,p4]= find_project_edge(pbx,pby);

                %res(res_index) = counter_edge_pixel(p1,p2,grey_img)+counter_edge_pixel(p3,p4,grey_img);
                
                res(res_index) = counter_density(pbx,pby,grey_img);
                
                res_index = res_index+1;
            end
        
    end
    
    [maxVal,idx] = max(res);
    
    %decompose idx to i and k
    
    idx_i=1;
    idx_k=1;% bottom param
   
    for jk=1:size_j*size_k
        for i=1:11
            if(idx ==((jk-1)*11+i))
                idx_i=i;
                idx_k=jk;
                break;
            end
        end      
    end
    
    raduis_results = from_i+(idx_i-1)*step_i;
    tb_results=zeros(1,2);
    tb_results(1)=top_param;
    tb_results(2)=from_k+(idx_k-1)*step_k;
end
