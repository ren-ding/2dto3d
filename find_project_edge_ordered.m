%p1,p2 are one side of two points,
%p3,p4 are another side of two points
%p2---j1---p3    
%|          | 
% |          | 
%  |          | 
%   p1---j2---p4 
function [p1,p2,p3,p4]= find_project_edge_ordered(j1,j2,pbx,pby)
    dists = zeros(1,length(pbx)-1);
    
    for i=1:length(pbx)-1
        X = [pbx(i),pby(i);pbx(i+1),pby(i+1)];
        dists(i) = pdist(X,'euclidean');
    end
    
    [sorted_dists,sorted_idx] = sort(dists,'descend');
    p1 = [pbx(sorted_idx(1)),pby(sorted_idx(1))];
    p2 = [pbx(sorted_idx(1)+1),pby(sorted_idx(1)+1)];
    p3 = [pbx(sorted_idx(2)),pby(sorted_idx(2))];
    p4 = [pbx(sorted_idx(2)+1),pby(sorted_idx(2)+1)];
    
    vec=j1-j2;
    v1 = p2-p1;
    a=acosd(dot(v1,vec)/(norm(v1)*norm(vec)));
    if(a>90 ||a <-90)
        p1_temp = p1;
        p4_temp = p4;
        p1=p2;
        p4=p3;
        p2=p1_temp;
        p3=p4_temp;
    end
    
    if ( (j2(1) - j1(1))*(p1(2) - j1(2)) - (j2(2) - j1(2))*(p1(1) - j1(1))<0)
        p1_temp = p1;
        p2_temp = p2;
        p1=p4;
        p2=p3;
        p3=p2_temp;
        p4=p1_temp;
    end

%     x_ps = [min(p1(2),p2(2)),min(p3(2),p4(2))];
%     [sorted_dists,sorted_idx] = sort(x_ps,'ascend');
%     if (sorted_idx(1)==2)
%         p1_temp = p1;
%         p2_temp = p2;
%         p1 = p3;
%         p2 = p4;
%         p3=p1_temp;
%         p4=p2_temp;
%     end;
%     y_ps = [p1(1),p2(1)];
%     [sorted_dists,sorted_idx] = sort(x_ps,'ascend');
%     if (sorted_idx(1)==1)
%         p1_temp=p1;p1=p2;p2=p1_temp;
%         p3_temp=p3;p3=p4;p4=p3_temp;        
%     end;
end
