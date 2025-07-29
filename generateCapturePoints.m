function Points = generateCapturePoints(lat, lon, h, captureDist, gridSpacing)
%GENERATECAPTUREPOINTS Returns wall & roof grid points for capture (no plotting)
%   Output:
%     Points : m×3 [x, y, z] ENU coordinates

if nargin < 4
    captureDist = 10;
end
if nargin < 5
    gridSpacing = 2*captureDist*tan(deg2rad(71)/2)*(1-0.8);
end

% GPS → ENU 변환
lat0 = mean(lat); lon0 = mean(lon); R = 6371000;
x = (lon - lon0)*pi/180*R*cos(lat0*pi/180);
y = (lat - lat0)*pi/180*R;

% 벽 포인트 계산
ptsW = [];
n = numel(x);
for i = 1:n
    j = mod(i,n)+1;
    xi = [x(i), x(j)]; yi = [y(i), y(j)];
    d = [xi(2)-xi(1); yi(2)-yi(1)]; L = norm(d);
    tdir = d/L; nrm = [tdir(2); -tdir(1)];
    t = 0:gridSpacing:L;
    zLevels = 0:gridSpacing:h;
    [T,Z] = meshgrid(t, zLevels);
    Xi = xi(1) + tdir(1)*T;
    Yi = yi(1) + tdir(2)*T;
    Xw = Xi + nrm(1)*captureDist;
    Yw = Yi + nrm(2)*captureDist;
    Zw = Z;
    ptsW = [ptsW; Xw(:), Yw(:), Zw(:)];
end

% 지붕 포인트 계산
xmin = min(x); xmax = max(x);
ymin = min(y); ymax = max(y);
[Xr, Yr] = meshgrid(xmin:gridSpacing:xmax, ymin:gridSpacing:ymax);
in = inpolygon(Xr(:), Yr(:), x, y);
Xr = Xr(in); Yr = Yr(in);
Zr = h + captureDist;

ptsR = [Xr, Yr, Zr*ones(size(Xr))];

Points = [ptsW; ptsR];
end
