function route = solve_cheapest(Points, startPos)
    % Points: n×d, startPos: 1×d
    unvisited = Points;
    % 1) 초기 순환: start -> 첫점 -> start
    route = [startPos; unvisited(1,:); startPos];
    unvisited(1,:) = [];
    
    while ~isempty(unvisited)
        bestInc = inf;
        bestPt  = [];
        bestPos = 0;
        % 2) 모든 미방문 점에 대해
        for i = 1:size(unvisited,1)
            pt = unvisited(i,:);
            % 2-1) 순회 상의 모든 간격 j→j+1에 삽입 시 늘어나는 거리
            for j = 1:size(route,1)-1
                inc = norm(route(j,:) - pt) + norm(pt - route(j+1,:)) ...
                      - norm(route(j,:) - route(j+1,:));
                if inc < bestInc
                    bestInc = inc;
                    bestPt  = pt;
                    bestPos = j+1;  % 삽입할 인덱스
                end
            end
        end
        % 2-2) 최적 삽입
        route = [route(1:bestPos-1,:); bestPt; route(bestPos:end,:)];
        % 2-3) 삽입된 점은 제거
        unvisited(ismember(unvisited, bestPt, 'rows'), :) = [];
    end
    
    % (선택) 순환 닫기: 이미 route가 startPos→…→startPos 형태라면 필요 없음
end
