% cluster_hierarchical.m
function pointGroups = cluster_hierarchical(Points, numDrones, ~)
%CLUSTER_HIERARCHICAL Agglomerative hierarchical clustering (Ward’s method)
%   Inputs:
%     Points    : m×3 array of [x,y,z] capture points
%     numDrones : number of clusters
%     ~         : startPos (unused)
%   Output:
%     pointGroups : 1×numDrones cell array

    % 1) 거리 기반 덴드로그램 생성 (Ward linkage 권장)
    Z = linkage(Points, 'ward');
    % 2) 덴드로그램을 잘라서 k 클러스터 얻기
    idx = cluster(Z, 'maxclust', numDrones);

    % 3) 결과 묶기
    pointGroups = cell(numDrones,1);
    for k = 1:numDrones
        pointGroups{k} = Points(idx==k, :);
    end
end