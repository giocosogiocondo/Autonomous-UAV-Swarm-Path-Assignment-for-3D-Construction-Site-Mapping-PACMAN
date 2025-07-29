function pointGroups = cluster_rhierarchical(Points, numDrones, ~)
%CLUSTER_HIERARCHICAL_BALANCED
%  1) Ward 계층적 클러스터링으로 numDrones개 묶음의 초깃값(centroid) 구함
%  2) 각 클러스터 당 ⌊m/K⌋~⌈m/K⌉ 개 제약(capacity) 계산
%  3) 모든 포인트→초깃 centroids 거리 행렬 D에 대해
%     capacity를 지키며 “가까운 클러스터부터” 할당(balancedAssign)

    m = size(Points,1);
    %--- capacity 계산: 균등 분배 (차이는 최대 1) ---
    baseSize = floor(m/numDrones);
    extra    = m - baseSize*numDrones;
    capacities = baseSize * ones(numDrones,1);
    capacities(1:extra) = capacities(1:extra) + 1;

    %--- 1) hierarchical로 초기 centroids 구하기 ---
    Z = linkage(Points, 'ward');
    idx0 = cluster(Z, 'maxclust', numDrones);
    centroids = zeros(numDrones, 3);
    for k = 1:numDrones
        centroids(k,:) = mean(Points(idx0==k,:), 1);
    end

    %--- 2) 거리 행렬, 3) 균등 할당 ---
    D = pdist2(Points, centroids);         % m×K
    idx = balancedAssign(D, capacities);   % m×1

    %--- 결과 묶기 ---
    pointGroups = cell(numDrones,1);
    for k = 1:numDrones
        pointGroups{k} = Points(idx==k, :);
    end
end

function idx = balancedAssign(D, capacities)
%BALANCEDASSIGN 거리 행렬 D, capacity 제약 하에
%  “거리 오름차순”으로 포인트→클러스터 할당
    [m, K] = size(D);
    [I,J] = ndgrid(1:m, 1:K);
    pairs = [I(:), J(:)];
    [~, ord] = sort(D(:));  % 가장 작은 거리부터
    idx = zeros(m,1);
    caps = capacities;
    for t = ord.'
        pt = pairs(t,1);
        cl = pairs(t,2);
        if idx(pt)==0 && caps(cl)>0
            idx(pt) = cl;
            caps(cl) = caps(cl) - 1;
        end
        if all(idx>0), break; end
    end
end
