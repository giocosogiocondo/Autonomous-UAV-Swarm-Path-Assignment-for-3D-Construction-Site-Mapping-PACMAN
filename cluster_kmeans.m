function pointGroups = cluster_kmeans(Points, numDrones, startPos)
%CLUSTER_KMEANS Assigns points to drones using k-means clustering
%   Inputs:
%     Points      : m×3 array of [x, y, z] capture points
%     numDrones   : number of drones (clusters)
%     startPos    : 1×3 vector, all drones start from same location
%   Output:
%     pointGroups : 1×numDrones cell array, each cell contains [x, y, z] points

% 안전: 모든 포인트가 3D 벡터인지 확인
assert(size(Points,2) == 3, 'Points must be an m×3 matrix.');

% 클러스터링용 초기 중심 설정 (기존 로직 그대로)
opts = statset('MaxIter',1000);
initIdx = randperm(size(Points,1), numDrones);
initCentroids = Points(initIdx, :);

% k-means 클러스터링
[idx, ~] = kmeans(Points, numDrones, 'Start', initCentroids, 'Options', opts);

% 결과 분배
pointGroups = cell(numDrones, 1);
for k = 1:numDrones
    pointGroups{k} = Points(idx == k, :);
end
end
