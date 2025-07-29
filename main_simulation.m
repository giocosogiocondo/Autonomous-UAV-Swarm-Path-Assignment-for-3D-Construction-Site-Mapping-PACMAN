% main_simulation.m
% 멀티 드론 최적 경로 시뮬레이션 (Min-Max 거리 최적화)
% -----------------------------------------------------------
% NOTE: MATLAB R2016b 이상 (스크립트 내 로컬 함수 지원)

clc; clear; close all;

%% 1. 드론 수, 시작점 및 알고리즘 설정
numDrones   = 5;                   % 사용할 드론 수
startPos    = [-50, 50, 0];        % 모든 드론이 시작할 위치 (ENU 좌표)
clusterMethod = 'rkmeans';          % 'kmeans' / 'rkmeans' / 'hierarchical' / 'rhierarchical'
routeMethod = 'greedy';          % 'greedy' / 'cheapest' / 'ga' / 'ant' / '2opt'

%% 2. 건물 포인트 생성
lat = [37.503990; 37.503030; 37.503030; 37.503990];
lon = [126.956871; 126.956871; 126.957385; 126.957385];
%고려대 공학관
%lat = [37.584046; 37.583690; 37.583521; 37.583253; 37.583697; 37.583936; 37.583857; 37.584186];
%lon = [127.024861; 127.025091; 127.024700; 127.024874; 127.025862; 127.025695; 127.025444; 127.025165];
h   = 30;                          % 높이 [m]
captureDist = 10;                  % 카메라-대상 거리 [m]
gridSpacing  = 2*captureDist*tan(deg2rad(71)/2)*(1-0.8);

Points = generateCapturePoints(lat, lon, h, captureDist, gridSpacing);

% (선택) 건물과 포인트만 시각화
visualizeScene(lat, lon, h, Points);   % 경로 / 시작점 미포함

%% 3. 드론별 포인트 분배
switch lower(clusterMethod)
    case 'kmeans'
        % k-means 기반 분배
        pointGroups = cluster_kmeans(Points, numDrones, startPos);
    case 'rkmeans'
        % Restricted K-means 분할 (클러스터링 수를 비슷하게)
        pointGroups = cluster_rkmeans(Points, numDrones, startPos);
    case 'hierarchical'
        % Hierarchical clustering 
        pointGroups = cluster_hierarchical(Points, numDrones, startPos);
    case 'rhierarchical'
        % Restricted Hierarchical 분할 (클러스터링 수를 비슷하게)
        pointGroups = cluster_rhierarchical(Points, numDrones, startPos);
    
    otherwise
        error('알 수 없는 clusterMethod: %s', clusterMethod);
end

%% 4. 드론별 경로 최적화 (시작점 포함)
droneRoutes = cell(numDrones, 1);
for i = 1:numDrones
    switch lower(routeMethod)
        case 'greedy'
            % Greedy Method
            droneRoutes{i} = solve_greedy(pointGroups{i}, startPos);
        case 'cheapest'
            % Cheapest Insertion
            droneRoutes{i} = solve_cheapest(pointGroups{i}, startPos);
        case 'ga'
            droneRoutes{i} = solve_ga(pointGroups{i}, startPos);
        case 'ant'
            droneRoutes{i} = solve_ant(pointGroups{i}, startPos);      
        case '2opt'
            droneRoutes{i} = solve_2opt(pointGroups{i}, startPos);  
        otherwise
            error('Unknown route method: %s', routeMethod);
    end
end

%% 5. 경로 길이 계산 (최장 경로 식별)
routeLengths = zeros(numDrones, 1);
for i = 1:numDrones
    routeLengths(i) = calculateRouteLength(droneRoutes{i});
end

%--- 주요 통계 지표 계산
maxLength      = max(routeLengths);
minLength      = min(routeLengths);
totalLength    = sum(routeLengths);
stdLength      = std(routeLengths, 1);                  % 모집단 기준 표준편차
imbalanceRatio = (maxLength - minLength) / maxLength;   % 경로 길이 불균형 비율

maxLength = max(routeLengths);

%% 6. 시각화 (건물 + 포인트 + 드론 경로 + 시작점)
visualizeScene(lat, lon, h, Points, startPos, droneRoutes);

%--- 결과 출력
fprintf('📍 드론별 경로 길이 (m):\n');
for i = 1:numDrones
    fprintf('  드론 %d: %.2f m\n', i, routeLengths(i));
end
fprintf('🔻 최장 경로 거리 (Min–Max): %.2f m\n\n', maxLength);
fprintf('➤ 총합             : %.2f m\n', totalLength);
fprintf('➤ 표준편차         : %.2f m\n', stdLength);
fprintf('➤ 불균형 비율      : %.4f\n', imbalanceRatio);

%% 8. 경로 애니메이션 재생 (speed = 0.05초 간격)          ----     아직 수정중...
%replayDroneRoutes(droneRoutes, startPos, lat, lon, h, Points, 0.05);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 로컬 함수
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function visualizeScene(lat, lon, h, Points, startPos, droneRoutes)
%VISUALIZESCENE 3-D 건물, 캡처 포인트, (옵션) 드론 경로를 시각화한다.
%   필수 입력:
%     lat, lon : 건물 꼭짓점의 위도/경도 (Nx1)
%     h        : 건물 높이 [m]
%     Points   : Mx3 ENU 캡처 포인트
%   선택 입력:
%     startPos    : 1x3 ENU 시작 위치 ([] 가능)
%     droneRoutes : 1xK cell, 각 cell = Px3 ENU 경로 ([] 가능)

if nargin < 5 || isempty(startPos),   startPos   = []; end
if nargin < 6 || isempty(droneRoutes), droneRoutes = {}; end

figure; hold on;

%% 1) 건물 패치 및 포인트 ----------------------------------------------------------------
lat0 = mean(lat); lon0 = mean(lon); R = 6371000;   % 지구 반경 [m]
xe = (lon - lon0) * pi/180 * R * cosd(lat0);
ye = (lat - lat0) * pi/180 * R;
z0 = zeros(size(xe)); z1 = h * ones(size(xe));
N = numel(xe);

% 바닥 & 지붕 패치
patch(xe, ye, z0, 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'DisplayName', 'Base');
patch(xe, ye, z1, 'red',  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'DisplayName', 'Roof');
% 벽
for j = 1:N
    k = mod(j, N) + 1;
    patch([xe(j) xe(k) xe(k) xe(j)], [ye(j) ye(k) ye(k) ye(j)], ...
          [0 0 h h], 'green', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'DisplayName', 'Walls');
end

% 캡처 포인트
scatter3(Points(:,1), Points(:,2), Points(:,3), 20, 'm', 'filled', 'DisplayName', 'Capture Points');

%% 2) (옵션) 드론 경로 & 시작점 -------------------------------------------------------------
if ~isempty(droneRoutes)
    C = lines(numel(droneRoutes));
    for k = 1:numel(droneRoutes)
        r = droneRoutes{k};
        plot3(r(:,1), r(:,2), r(:,3), '-', 'LineWidth', 2, 'Color', C(k,:), ...
              'DisplayName', sprintf('Drone %d', k));
        text(r(end,1), r(end,2), r(end,3), sprintf('D%d', k), 'Color', C(k,:), ...
             'FontWeight', 'bold', 'FontSize', 9);
    end
    if ~isempty(startPos)
        scatter3(startPos(1), startPos(2), startPos(3), 100, 'k', 'filled', 'DisplayName', 'Start');
        text(startPos(1), startPos(2), startPos(3), 'Start', 'VerticalAlignment', 'bottom', ...
             'HorizontalAlignment', 'right');
    end
end

%% 3) 공통 시각화 옵션 ----------------------------------------------------------------------
axis equal; grid on;
xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
view(3);
title('Building, Capture Points and Drone Routes');
legend('Location', 'bestoutside');
end
