function pointGroups = cluster_rkmeans(Points, numDrones, startPos)
%CLUSTER_KMEANS_BALANCED Size‐constrained k‐means: 균등 개수 보장
%   Inputs:
%     Points      : m×3 array of [x, y, z]
%     numDrones   : 클러스터 수
%     startPos    : 1×3 시작 위치 (초기 중심 분산용)
%   Output:
%     pointGroups : 1×numDrones cell, 각 셀에 균등 할당된 포인트

    m = size(Points,1);
    % 1) 각 클러스터가 가져야 할 개수 계산
    baseSize = floor(m/numDrones);
    extra    = m - baseSize*numDrones;
    % capacities: 첫 extra개는 baseSize+1, 나머지는 baseSize
    capacities = baseSize * ones(numDrones,1);
    capacities(1:extra) = capacities(1:extra) + 1;

    % 2) 초기 중심: 기존 로직 그대로 (혹은 랜덤)
    initIdx      = randperm(m, numDrones);
    centroids    = Points(initIdx, :);

    opts.MaxIter = 100;
    tol          = 1e-4;
    prevCentroids = inf(size(centroids));

    % 3) 반복: 중심 업데이트 + 균등 할당
    while norm(prevCentroids - centroids, 'fro') > tol && opts.MaxIter > 0
        prevCentroids = centroids;
        opts.MaxIter = opts.MaxIter - 1;

        % 3-1) 거리 행렬
        D = pdist2(Points, centroids);

        % 3-2) 균형 할당
        idx = balancedAssign(D, capacities);

        % 3-3) 중심 재계산
        for k = 1:numDrones
            centroids(k,:) = mean(Points(idx==k,:),1);
        end
    end

    % 4) 최종 분할
    pointGroups = cell(numDrones,1);
    for k = 1:numDrones
        pointGroups{k} = Points(idx==k, :);
    end
end

function idx = balancedAssign(D, capacities)
% 각 포인트-클러스터 쌍을 거리 기준 오름차순 정렬 → capacity 남은 곳에 분배
    [m, k] = size(D);
    % 1) 모든 (point,cluster) 쌍 나열
    [I,J]  = ndgrid(1:m,1:k);
    pairs = [I(:), J(:)];
    dists = D(:);
    [~, ord] = sort(dists);

    idx        = zeros(m,1);
    caps       = capacities;  % 로컬 복사
    for t = ord.'
        pt = pairs(t,1);
        cl = pairs(t,2);
        if idx(pt)==0 && caps(cl)>0
            idx(pt) = cl;
            caps(cl) = caps(cl) - 1;
        end
        % 모두 할당되면 종료
        if all(idx>0), break; end
    end
end
