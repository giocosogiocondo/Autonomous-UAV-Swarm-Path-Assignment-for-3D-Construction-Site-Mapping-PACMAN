function replayDroneRoutes(droneRoutes, startPos, lat, lon, h, Points, speed)
%REPLAYDRONEROUTES Animates drone movement in 3D over building and capture points
%   Inputs:
%     droneRoutes : cell array of [x, y, z] route points
%     startPos    : 1×3 vector (ENU)
%     lat, lon    : building footprint (degrees)
%     h           : building height (m)
%     Points      : all capture points (m×3, ENU)
%     speed       : pause duration per step (seconds)

if nargin < 7
    speed = 0.05;  % default playback speed
end

figure; hold on;

%% 1. 건물 시각화
lat0 = mean(lat); lon0 = mean(lon); R = 6371000;
x = (lon - lon0)*pi/180*R*cos(lat0*pi/180);
y = (lat - lat0)*pi/180*R;
z0 = zeros(size(x)); z1 = h*ones(size(x));
n = numel(x);

patch(x, y, z0, 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'k');   % base
patch(x, y, z1, 'red',  'FaceAlpha', 0.3, 'EdgeColor', 'k');   % roof
for i = 1:n
    j = mod(i, n) + 1;
    patch([x(i) x(j) x(j) x(i)], ...
          [y(i) y(j) y(j) y(i)], ...
          [0     0     h     h], ...
          'green', 'FaceAlpha', 0.3, 'EdgeColor', 'k');        % walls
end

%% 2. 포인트 시각화
scatter3(Points(:,1), Points(:,2), Points(:,3), 15, 'm', 'filled');

%% 3. 시작점 표시
scatter3(startPos(1), startPos(2), startPos(3), 80, 'k', 'filled');
text(startPos(1), startPos(2), startPos(3), 'Start', ...
    'VerticalAlignment','bottom','HorizontalAlignment','right');

%% 4. 드론 위치 초기화
numDrones = numel(droneRoutes);
colors = lines(numDrones);
droneMarkers = gobjects(numDrones, 1);
droneTrails = gobjects(numDrones, 1);

for i = 1:numDrones
    route = droneRoutes{i};

    % 보정: z가 모두 0인 경우 기본 고도 부여
    if all(route(:,3) == 0)
        warning("Drone %d의 고도 정보가 없음 → z에 높이 보정", i);
        route(:,3) = linspace(5, 20, size(route,1));
        droneRoutes{i} = route;
    end

    p = route(1,:);
    droneMarkers(i) = plot3(p(1), p(2), p(3), 'o', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', colors(i,:), ...
        'MarkerEdgeColor', 'k');
    
    % 경로 라인 초기화
    droneTrails(i) = plot3(route(:,1), route(:,2), route(:,3), '--', ...
        'Color', colors(i,:), 'LineWidth', 1);
end

%% 5. 경로 애니메이션
maxLen = max(cellfun(@(r) size(r,1), droneRoutes));
for step = 2:maxLen
    for i = 1:numDrones
        route = droneRoutes{i};
        if step <= size(route,1)
            p = route(step,:);
            set(droneMarkers(i), 'XData', p(1), 'YData', p(2), 'ZData', p(3));
        end
    end
    pause(speed);
end

%% 6. 시각화 마무리 설정
xlabel('East (m)');
ylabel('North (m)');
zlabel('Height (m)');
title('🛩️ Drone Route Replay (True 3D View)');
grid on;

axis vis3d;     % 3D 비율 고정
view(45, 30);   % ↖️↘️ 입체 시점

% Z축 강제 범위 설정
allZ = cellfun(@(r) r(:,3), droneRoutes, 'UniformOutput', false);
zMax = max(cellfun(@max, allZ));
zlim([0, zMax + 10]);

end
