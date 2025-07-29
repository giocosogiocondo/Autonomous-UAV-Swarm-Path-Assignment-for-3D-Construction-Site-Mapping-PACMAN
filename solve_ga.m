function route = solve_ga(Points, startPos)
%SOLVE_GA  Solve TSP using a custom Genetic Algorithm
%   Inputs:
%     Points   : n×3 array of [x, y, z] capture points
%     startPos : 1×3 vector, start position of drone
%   Output:
%     route    : (n+1)×3 array, visiting all Points in GA-optimal order

    % Set GA parameters
    popSize      = 50;    % Population size
    numGens      = 100;   % Number of generations
    eliteCount   = 5;     % Number of elites to carry over
    tourSize     = 3;     % Tournament size
    crossoverRate= 0.8;   % Crossover probability
    mutationRate = 0.2;   % Mutation probability

    % Number of points
    nPoints = size(Points,1);

    % Precompute distances
    D_pp = pdist2(Points, Points);          % point-to-point distances
    D_sp = pdist2(startPos, Points);        % start-to-point distances

    % Fitness function: total path length from start through permutation
    function L = routeLength(perm)
        L = D_sp(perm(1));
        for ii = 1:nPoints-1
            L = L + D_pp(perm(ii), perm(ii+1));
        end
    end

    % Initialize population: random permutations
    pop = zeros(popSize, nPoints);
    for i = 1:popSize
        pop(i,:) = randperm(nPoints);
    end

    % Evaluate initial fitness
    fitness = zeros(popSize,1);
    for i = 1:popSize
        fitness(i) = routeLength(pop(i,:));
    end

    % GA main loop
    for gen = 1:numGens
        % Sort population by fitness (ascending)
        [fitness, idxSort] = sort(fitness);
        pop = pop(idxSort, :);

        % Elitism: carry over top elites
        newPop = pop(1:eliteCount, :);

        % Create offspring
        while size(newPop,1) < popSize
            % Tournament selection
            parent1 = tournamentSelect(pop, fitness, tourSize);
            parent2 = tournamentSelect(pop, fitness, tourSize);

            % Crossover
            if rand < crossoverRate
                child = orderedCrossover(parent1, parent2);
            else
                child = parent1;
            end

            % Mutation: swap two genes
            if rand < mutationRate
                child = swapMutation(child);
            end

            newPop(end+1,:) = child; %#ok<AGROW>
        end

        pop = newPop;

        % Recompute fitness
        for i = 1:popSize
            fitness(i) = routeLength(pop(i,:));
        end
    end

    % Final best permutation
    [~, bestIdx] = min(fitness);
    bestPerm = pop(bestIdx, :);

    % Build output route
    route = [startPos; Points(bestPerm,:)];
end

%% Helper Functions
function selected = tournamentSelect(pop, fitness, k)
    % Randomly pick k individuals, return the best (smallest fitness)
    idx = randperm(size(pop,1), k);
    [~, best] = min(fitness(idx));
    selected = pop(idx(best), :);
end

function child = orderedCrossover(p1, p2)
    n = numel(p1);
    % Choose two cut points
    c1 = randi(n);
    c2 = randi(n);
    if c1 > c2, [c1,c2] = deal(c2,c1); end
    child = nan(1, n);
    % Copy segment from parent1
    child(c1:c2) = p1(c1:c2);
    % Fill remainder from parent2 in order
    fillPos = [c2+1:n, 1:c1-1];
    p2seq = p2(~ismember(p2, child));
    child(fillPos) = p2seq;
end

function perm = swapMutation(perm)
    n = numel(perm);
    i = randi(n);
    j = randi(n);
    perm([i j]) = perm([j i]);
end