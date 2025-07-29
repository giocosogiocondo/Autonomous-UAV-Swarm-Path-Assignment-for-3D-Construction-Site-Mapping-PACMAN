function route = solve_ga_gpu(Points, startPos)
%SOLVE_GA_GPU  Solve TSP using GA on the GPU
%   Inputs:
%     Points   : n×3 array of [x, y, z] capture points
%     startPos : 1×3 vector, start position of drone
%   Output:
%     route    : (n+1)×3 array, visiting all Points in GA-optimal order

    % 1) GPU로 올리기
    Points   = gpuArray(Points);
    startPos = gpuArray(startPos);

    % GA 파라미터
    popSize      = 50;
    numGens      = 1000;
    eliteCount   = 5;
    tourSize     = 3;
    crossoverRate= 0.8;
    mutationRate = 0.2;
    nPoints = size(Points,1);

    % 2) 거리 행렬 미리 계산 (GPU 내부에서)
    D_pp = pdist2(Points, Points);      % point–point
    D_sp = pdist2(startPos, Points);    % start–point

    % 3) 초기 개체군: CPU에서 만든 뒤 GPU로 옮기기
    popCPU = zeros(popSize, nPoints, 'uint32');
    for i = 1:popSize
        popCPU(i,:) = uint32(randperm(nPoints));
    end
    pop = gpuArray(popCPU);

    % 4) GPU상에서 fitness 계산을 위한 공간 확보
    fitness = gpuArray.zeros(popSize,1);

    % Fitness 함수 (GPU array 지원)
    function L = routeLength(perm)
        L = D_sp(perm(1));
        for ii = 1:nPoints-1
            L = L + D_pp(perm(ii), perm(ii+1));
        end
    end

    % 초기 fitness
    for i = 1:popSize
        fitness(i) = routeLength(pop(i,:));
    end

    % 5) GA 메인 루프 (모두 GPU상에서)
    for gen = 1:numGens
        % 정렬
        [fitness, idxSort] = sort(fitness);
        pop = pop(idxSort, :);

        % 엘리트 보존
        newPop = pop(1:eliteCount, :);

        while size(newPop,1) < popSize
            p1 = tournamentSelect(pop, fitness, tourSize);
            p2 = tournamentSelect(pop, fitness, tourSize);
            if rand < crossoverRate
                child = orderedCrossover(p1, p2);
            else
                child = p1;
            end
            if rand < mutationRate
                child = swapMutation(child);
            end
            newPop(end+1,:) = child; %#ok<AGROW>
        end
        pop = newPop;

        for i = 1:popSize
            fitness(i) = routeLength(pop(i,:));
        end
    end

    % 6) 최적 해를 CPU로 가져와서 경로 생성
    [~, bestIdx] = min(gather(fitness));
    bestPerm = gather(pop(bestIdx, :));
    route = [gather(startPos); gather(Points(bestPerm,:))];
end

%% Helper functions (CPU/GPU 모두 호환)
function sel = tournamentSelect(pop, fitness, k)
    idx = randi([1 size(pop,1)], [k,1], 'gpuArray');
    [~, b] = min(fitness(idx));
    sel = pop(idx(b), :);
end

function child = orderedCrossover(p1, p2)
    n = numel(p1);
    c1 = randi(n,'gpuArray'); c2 = randi(n,'gpuArray');
    if c1>c2, [c1,c2] = deal(c2,c1); end
    child = zeros(1,n,'like',p1);
    child(c1:c2) = p1(c1:c2);
    fillPos = [c2+1:n,1:c1-1];
    p2seq = p2(~ismember(p2,child));
    child(fillPos) = p2seq;
end

function perm = swapMutation(perm)
    n = numel(perm);
    i = randi(n,'gpuArray'); j = randi(n,'gpuArray');
    perm([i j]) = perm([j i]);
end
