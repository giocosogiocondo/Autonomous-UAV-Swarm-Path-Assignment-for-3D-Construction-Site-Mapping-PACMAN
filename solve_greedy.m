function route = solve_greedy(Points, startPos)
%SOLVETSPGREEDY Solves a greedy TSP starting from startPos
%   Inputs:
%     Points   : n×3 array of [x, y, z] coordinates
%     startPos : 1×3 vector (start position of drone)
%   Output:
%     route    : (n+1)×3 array, starting from startPos and visiting all points

% 초기화
numPoints = size(Points, 1);
visited = false(numPoints, 1);
route = startPos;  % 시작점 포함

current = startPos;

for i = 1:numPoints
    % 방문하지 않은 점들 중 현재 위치에서 가장 가까운 점 선택
    dists = vecnorm(Points - current, 2, 2);  % 모든 점과의 거리
    dists(visited) = inf;  % 이미 방문한 점 제외
    [~, idx] = min(dists);
    
    visited(idx) = true;
    current = Points(idx, :);
    route = [route; current];  % 경로에 추가
end
end
