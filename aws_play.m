function aws_play(R, opts)
%AWS_PLAY  Animate a simulation result in 2-D (top view) or 3-D.
%   aws_play(R)
%   aws_play(R, struct('view', '3d', 'speed', 1, 'gif', 'demo.gif'))
%
%   opts.view    '2d' (default) or '3d'
%   opts.speed   playback speed factor (1 = real time, 2 = twice as fast)
%   opts.gif     file name to record an animated GIF ('' = do not record)
%   opts.skip    draw every skip-th simulation step (default 2 -> 10 fps)
%
%   Colours:  blue = approaching, amber = stopped/waiting, green = crossing,
%             grey = cleared, red = human driver running the sign.
%   Plain patch graphics only: runs fine on integrated graphics.

if nargin < 2, opts = struct(); end
if ~isfield(opts, 'view'),  opts.view  = '2d'; end
if ~isfield(opts, 'speed'), opts.speed = 1;    end
if ~isfield(opts, 'gif'),   opts.gif   = '';   end
if ~isfield(opts, 'skip'),  opts.skip  = 2;    end
is3d = strcmpi(opts.view, '3d');

P = R.P;
N = size(R.X, 2);
w = P.w;  b = P.b;  Lv = 34;

colApproach = [0.20 0.45 0.90];
colWait     = [1.00 0.70 0.10];
colGo       = [0.20 0.75 0.30];
colExit     = [0.62 0.62 0.67];
colRunner   = [0.85 0.15 0.15];
stateCol    = [colApproach; colWait; colGo; colExit];

fig = figure('Color', 'w', 'Name', 'All-way stop', 'NumberTitle', 'off');
ax  = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
set(ax, 'XLim', [-Lv Lv], 'YLim', [-Lv Lv], 'ZLim', [0 12], ...
        'XTick', [], 'YTick', [], 'ZTick', [], 'Box', 'on');

% --- static scene ----------------------------------------------------------
flat(ax, [-Lv Lv Lv -Lv], [-Lv -Lv Lv Lv], 0.00, [0.55 0.72 0.50]);      % grass
flat(ax, [-Lv Lv Lv -Lv], [-w -w w w],     0.01, [0.33 0.33 0.36]);      % E-W road
flat(ax, [-w w w -w],     [-Lv -Lv Lv Lv], 0.01, [0.33 0.33 0.36]);      % N-S road
for a = 0:3
    c = cos(a*pi/2);  s = sin(a*pi/2);
    rot = [c s; -s c];
    % stop line (across the incoming lane)
    q = [0 -b; w -b; w -b + 0.45; 0 -b + 0.45]*rot;
    flat(ax, q(:, 1), q(:, 2), 0.03, [1 1 1]);
    % crosswalk stripes
    for xs = -w + 0.35:1.0:w - 0.6
        q = [xs -5.5; xs + 0.5 -5.5; xs + 0.5 -3.9; xs -3.9]*rot;
        flat(ax, q(:, 1), q(:, 2), 0.02, [0.92 0.92 0.92]);
    end
    % centre line
    q = [0 -Lv; 0 -b]*rot;
    plot3(ax, q(:, 1), q(:, 2), [0.03 0.03], '--', 'Color', [1 0.85 0.2], 'LineWidth', 1.2);
    % stop sign at the right-hand corner
    q  = [w + 1.2, -b - 0.5]*rot;
    th = (0:7)'*pi/4 + pi/8;
    if is3d
        plot3(ax, [q(1) q(1)], [q(2) q(2)], [0 2.2], 'Color', [0.4 0.4 0.4], 'LineWidth', 2);
        n = [0 -1]*rot;  tng = [1 0]*rot;       % sign faces the incoming driver
        patch('Parent', ax, 'XData', q(1) + 0.55*cos(th)*tng(1) + 0.02*n(1), ...
              'YData', q(2) + 0.55*cos(th)*tng(2) + 0.02*n(2), ...
              'ZData', 2.6 + 0.55*sin(th), 'FaceColor', [0.85 0.1 0.1], 'EdgeColor', 'w');
    else
        patch('Parent', ax, 'XData', q(1) + 0.9*cos(th), 'YData', q(2) + 0.9*sin(th), ...
              'ZData', 0.05*ones(8, 1), 'FaceColor', [0.85 0.1 0.1], 'EdgeColor', 'w');
    end
end

% --- cars ------------------------------------------------------------------
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
hl = P.carLen/2;  hw = P.carWid/2;
hBody = gobjects(1, N);  hCab = gobjects(1, N);  hTxt = gobjects(1, N);
for i = 1:N
    hBody(i) = patch('Parent', ax, 'Vertices', zeros(8, 3), 'Faces', faces, ...
        'FaceColor', colApproach, 'EdgeColor', [0.1 0.1 0.1], 'Visible', 'off');
    hCab(i)  = patch('Parent', ax, 'Vertices', zeros(8, 3), 'Faces', faces, ...
        'FaceColor', [0.15 0.18 0.25], 'EdgeColor', 'none', 'Visible', 'off');
    hTxt(i)  = text(0, 0, 3, sprintf('%d', i), 'Parent', ax, 'Color', 'w', ...
        'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'center', ...
        'Visible', 'off');
end
hPed = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.95 0.3 0.8], 'MarkerEdgeColor', 'k');

% legend (dummy patches)
lg = gobjects(1, 5);
lgCols = [colApproach; colWait; colGo; colExit; colRunner];
for q = 1:5
    lg(q) = patch('Parent', ax, 'XData', nan, 'YData', nan, 'FaceColor', lgCols(q, :));
end
legend(ax, lg, {'approaching', 'stopped / waiting', 'crossing', 'cleared', 'human, runs sign'}, ...
    'Location', 'northeastoutside');

if is3d
    view(ax, -32, 38);
else
    view(ax, 2);
end
hTitle = title(ax, '');

% --- playback --------------------------------------------------------------
K = numel(R.t);
firstFrame = true;
tic;
for k = 1:opts.skip:K
    if ~ishandle(fig), return, end             % window was closed
    for i = 1:N
        if isnan(R.X(k, i))
            set(hBody(i), 'Visible', 'off');
            set(hCab(i),  'Visible', 'off');
            set(hTxt(i),  'Visible', 'off');
            continue
        end
        x = R.X(k, i);  y = R.Y(k, i);  yaw = R.YAW(k, i);
        if R.isRunner(i)
            col = colRunner;
        else
            col = stateCol(min(max(R.ST(k, i), 1), 4), :);
        end
        set(hBody(i), 'Vertices', boxVerts(x, y, yaw, -hl, hl, hw, 0.25, 1.0), ...
            'FaceColor', col, 'Visible', 'on');
        set(hCab(i),  'Vertices', boxVerts(x, y, yaw, -0.60*hl, 0.20*hl, 0.85*hw, 1.0, 1.55), ...
            'Visible', 'on');
        set(hTxt(i),  'Position', [x + 0.6*hl*cos(yaw), y + 0.6*hl*sin(yaw), 2.2], 'Visible', 'on');
    end
    set(hPed, 'XData', R.PX(k, :), 'YData', R.PY(k, :), 'ZData', 1.0 + 0*R.PX(k, :));
    stk = R.ST(k, :);
    set(hTitle, 'String', sprintf('%s    t = %5.1f s    waiting: %d   crossing: %d   cleared: %d / %d', ...
        strrep(R.name, '_', ' '), R.t(k), sum(stk == 2), sum(stk == 3), sum(stk >= 4), N));
    drawnow;

    if ~isempty(opts.gif)
        fr = getframe(fig);
        [im, map] = rgb2ind(frame2im(fr), 128);
        if firstFrame
            imwrite(im, map, opts.gif, 'gif', 'LoopCount', inf, 'DelayTime', P.dt*opts.skip/opts.speed);
            firstFrame = false;
        else
            imwrite(im, map, opts.gif, 'gif', 'WriteMode', 'append', 'DelayTime', P.dt*opts.skip/opts.speed);
        end
    else
        lag = R.t(k)/opts.speed - toc;          % keep real-time pace
        if lag > 0, pause(lag); end
    end
end
end

% ============================================================================
function flat(ax, xv, yv, z, faceCol)
patch('Parent', ax, 'XData', xv(:), 'YData', yv(:), 'ZData', z*ones(numel(xv), 1), ...
      'FaceColor', faceCol, 'EdgeColor', 'none');
end

% ============================================================================
function Vt = boxVerts(x, y, yaw, x0, x1, hw, z0, z1)
% 8 corners of a box given in the car frame, moved to the world frame.
loc = [x0 -hw; x1 -hw; x1 hw; x0 hw];
c = cos(yaw);  s = sin(yaw);
xy = loc*[c s; -s c];
xy = [xy(:, 1) + x, xy(:, 2) + y];
Vt = [xy, z0*ones(4, 1); xy, z1*ones(4, 1)];
end
