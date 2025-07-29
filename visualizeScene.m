function visualizeScene(lat, lon, h, Points, startPos, droneRoutes)
if nargin < 5, startPos   = []; end
if nargin < 6, droneRoutes = {}; end

figure; hold on;

%% ── 1. 건물 & 포인트 ────────────────────────────────
% ENU 변환
lat0 = mean(lat); lon0 = mean(lon); R = 6371000;
x = (lon - lon0)*pi/180*R*cos(lat0*pi/180);
y = (lat - lat0)*pi/180*R;
z0 = zeros(size(x)); z1 = h*ones(size(x)); n = numel(x);

% 바닥·지붕·벽
patch(x, y, z0, 'blue','FaceAlpha',0.3,'EdgeColor','k');
patch(x, y, z1, 'red', 'FaceAlpha',0.3,'EdgeColor','k');
for i = 1:n
    j = mod(i,n)+1;
    patch([x(i) x(j) x(j) x(i)], [y(i) y(j) y(j) y(i)], ...
          [0 0 h h],'green','FaceAlpha',0.3,'EdgeColor','k');
end
% 포인트
scatter3(Points(:,1),Points(:,2),Points(:,3),20,'m','filled');

%% ── 2. (선택) 드론 경로 & 시작점 ───────────────────
if ~isempty(droneRoutes)
    colors = lines(numel(droneRoutes));
    for k = 1:numel(droneRoutes)
        r = droneRoutes{k};
        plot3(r(:,1),r(:,2),r(:,3),'-','LineWidth',2,'Color',colors(k,:));
        text(r(end,1),r(end,2),r(end,3),sprintf('Drone %d',k), ...
             'Color',colors(k,:),'FontWeight','bold','FontSize',10);
    end
    if ~isempty(startPos)
        scatter3(startPos(1),startPos(2),startPos(3),100,'k','filled');
        text(startPos(1),startPos(2),startPos(3),'Start', ...
             'VerticalAlignment','bottom','HorizontalAlignment','right');
    end
end

%% ── 3. 공통 설정 ────────────────────────────────────
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
title('Building, Capture Points and Drone Routes');
view(3);

leg = {'Base','Roof','Walls','Capture Points'};
if ~isempty(droneRoutes), leg{end+1}='Drone Routes'; end
if ~isempty(startPos),    leg{end+1}='Start';       end
legend(leg,'Location','best');
end
