function totalDist = calculateRouteLength(route)
%CALCULATEROUTELENGTH Computes total 3D path length
%   Input:
%     route     : n×3 matrix of ordered [x, y, z] points
%   Output:
%     totalDist : scalar total distance along route

% 차분 벡터 계산 (현재점 → 다음점)
diffs = diff(route, 1, 1);  % (n-1)×3

% 각 구간의 유클리드 거리 계산
dists = vecnorm(diffs, 2, 2);  % (n-1)×1

% 전체 거리 합산
totalDist = sum(dists);
end