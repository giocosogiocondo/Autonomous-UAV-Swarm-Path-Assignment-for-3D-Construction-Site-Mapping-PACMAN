function route = solve_ant(Points, startPos)
%SOLVE_ANT  Solve TSP using Ant Colony Optimization
%   Inputs:
%     Points   : n×3 array of [x, y, z] capture points
%     startPos : 1×3 vector, start position of drone
%   Output:
%     route    : (n+1)×3 array, visiting all Points in ACO-optimal order

    % Parameters
    nPoints     = size(Points,1);
    numAnts     = nPoints;       % number of ants
    numIters    = 3000;           % number of iterations
    alpha       = 1;             % pheromone importance
    beta        = 5;             % heuristic importance
    rho         = 0.1;           % pheromone evaporation rate
    Q           = 1;             % pheromone deposit factor

    % Build coordinate list including start
    coords = [startPos; Points];         % (nPoints+1)×3
    % Distance matrix
    distM = pdist2(coords, coords);
    % Initialize pheromone
    pher = ones(nPoints+1);
    % Heuristic: inverse distance (avoid self)
    heuristic = 1./(distM + eps);

    bestLen = inf;
    bestTour = [];

    for iter = 1:numIters
        tours = zeros(numAnts, nPoints);
        lens  = zeros(numAnts,1);
        for a = 1:numAnts
            unvisited = 2:nPoints+1;
            current   = 1;  % start index
            tour      = zeros(1, nPoints);
            for k = 1:nPoints
                probs = zeros(1, numel(unvisited));
                for j = 1:numel(unvisited)
                    next = unvisited(j);
                    probs(j) = (pher(current,next)^alpha) * (heuristic(current,next)^beta);
                end
                probs = probs./sum(probs);
                r = rand;
                cum = cumsum(probs);
                idx = find(cum >= r, 1);
                chosen = unvisited(idx);
                tour(k) = chosen;
                unvisited(idx) = [];
                current = chosen;
            end
            tours(a,:) = tour;
            % compute length from start through tour
            L = distM(1, tour(1));
            for k = 1:nPoints-1
                L = L + distM(tour(k), tour(k+1));
            end
            lens(a) = L;
        end
        % Update pheromone
        pher = (1-rho) * pher;
        for a = 1:numAnts
            tour = tours(a,:);
            L = lens(a);
            % from start to first
            i = 1; j = tour(1);
            pher(i,j) = pher(i,j) + Q/L;
            pher(j,i) = pher(i,j);
            % edges between points
            for k = 1:nPoints-1
                i = tour(k); j = tour(k+1);
                pher(i,j) = pher(i,j) + Q/L;
                pher(j,i) = pher(i,j);
            end
        end
        % track best
        [minL, idxMin] = min(lens);
        if minL < bestLen
            bestLen = minL;
            bestTour = tours(idxMin,:);
        end
    end

    % Build route output
    route = [startPos; Points(bestTour-1,:)];
end
