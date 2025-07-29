function route = solve_2opt(Points, startPos)
%SOLVE_GREEDY2OPT  NN + 2-opt TSP 휴리스틱
%   route = (n+1)×d, startPos → … → startPos
%
%   Inputs
%     Points   : n×d 실수 행렬  (d = 2,3,…)
%     startPos : 1×d 출발·귀환 좌표
% -------------------------------------------------------------------------
n  = size(Points,1);
d  = size(Points,2);
assert(size(startPos,2)==d, 'Dimension mismatch.');

%% 1) Nearest-Neighbor로 초기 투어 -----------------------------
visited = false(n,1);
route   = startPos;                % 행렬 (k×d)
current = startPos;

for k = 1:n
    % 아직 안 간 점들의 거리
    distVec        = vecnorm(Points - current, 2, 2);
    distVec(visited) = inf;
    [~, idx] = min(distVec);
    visited(idx) = true;
    current      = Points(idx,:);
    route        = [route; current];
end
route = [route; startPos];         % 귀환 ← (n+1)×d

%% 2) 2-opt 로컬 개선 ----------------------------------------
improved = true;
while improved
    improved = false;
    L = size(route,1)-1;           % 유효 간선 수 (마지막은 startPos)
    for i = 2 : L-2                % 시작·끝 고정
        for k = i+1 : L-1
            % 교차 전·후 비용 비교
            a = route(i-1,:);  b = route(i,:);
            c = route(k  ,:);  d = route(k+1,:);
            old = norm(a-b) + norm(c-d);
            new = norm(a-c) + norm(b-d);
            if new < old - 1e-10   % 개선 여유 (수치 안전)
                route(i:k,:) = flipud(route(i:k,:));  % 구간 뒤집기
                improved = true;
            end
        end
    end
end
end
