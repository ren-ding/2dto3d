%p1,p2 are one side of two points,
%p3,p4 are another side of two points
 
function [p1,p2,p3,p4]= find_project_edge(pbx,pby)
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
    
end
