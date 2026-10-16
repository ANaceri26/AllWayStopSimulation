%DEMO_LUT_CALIBRATION  Let an optimiser calibrate a lookup table by trial and error.
%   Idea from the GM talk "Accelerating ECU Calibration Using Reinforcement
%   Learning": start from a safe but poor baseline table and let an agent
%   improve it against a reward, while respecting what a lookup table is
%   allowed to look like.
%
%   The table calibrated here is the AV's stop-sign approach map:
%        distance to the stop line (m)  ->  target speed (m/s)
%   Reward = -(time to stop)  - comfort penalty (jerk, braking harder than
%   3 m/s^2)  - big penalty for overshooting the line or stopping short.
%
%   GM used a TD3 agent from Reinforcement Learning Toolbox.  To stay
%   toolbox-free this demo uses the cross-entropy method, a simple
%   reward-driven policy search: sample 40 candidate tables, keep the best 8,
%   move the sampling distribution toward them, repeat.
%   The "LUT constraint" problem from the talk is handled by construction:
%   the search works on the (non-negative) steps between breakpoints, so
%   every candidate table is monotonic and capped at the cruise speed.

rng(0);
bp     = [0 1 2 4 7 11 16 22 30 40];     % breakpoints: distance to line (m)
vMax   = 10;
base   = min(vMax, 0.3*bp);              % baseline calibration: timid ramp
nPop   = 40;  nElite = 8;  nIter = 40;

mu  = diff(base);
sig = 0.6*ones(1, numel(bp) - 1);
bestHist = zeros(nIter, 1);  meanHist = zeros(nIter, 1);
for it = 1:nIter
    th   = repmat(mu, nPop, 1) + repmat(sig, nPop, 1).*randn(nPop, numel(mu));
    cost = zeros(nPop, 1);
    for p = 1:nPop
        cost(p) = rollout(bp, lutFrom(th(p, :), vMax));
    end
    [cs, order] = sort(cost);
    elite = th(order(1:nElite), :);
    mu    = mean(elite, 1);
    sig   = std(elite, 0, 1) + 0.02;
    bestHist(it) = -cs(1);  meanHist(it) = -mean(cost);
end
cal = lutFrom(mu, vMax);

[c0, log0] = rollout(bp, base);
[c1, log1] = rollout(bp, cal);
fprintf('\n%-12s %10s %12s %14s %12s\n', '', 'reward', 'stop time', 'peak braking', 'stops at');
fprintf('%-12s %10.2f %10.2f s %10.2f m/s2 %10.2f m\n', 'baseline',   -c0, log0.t(end), -min(log0.a), log0.d(end));
fprintf('%-12s %10.2f %10.2f s %10.2f m/s2 %10.2f m\n', 'calibrated', -c1, log1.t(end), -min(log1.a), log1.d(end));
fprintf('\nCalibrated table:\n   distance (m): %s\n   speed  (m/s): %s\n', ...
    sprintf('%6.1f', bp), sprintf('%6.2f', cal));

figure('Color', 'w', 'Name', 'LUT calibration', 'NumberTitle', 'off');
subplot(1, 3, 1);
plot(bp, base, '-o', 'Color', [0.9 0.5 0.1], 'LineWidth', 1.5);  hold on;
plot(bp, cal,  '-s', 'Color', [0.5 0.2 0.7], 'LineWidth', 1.5);
dd = linspace(0, 40, 100);
plot(dd, min(vMax, sqrt(2*3.0*dd)), 'g--', 'LineWidth', 1);
grid on;  xlabel('distance to stop line (m)');  ylabel('target speed (m/s)');
legend({'baseline', 'calibrated', 'physics limit (3 m/s^2)'}, 'Location', 'southeast');
title('Lookup table');
subplot(1, 3, 2);
plot(log0.t, log0.v, 'Color', [0.9 0.5 0.1], 'LineWidth', 1.5);  hold on;
plot(log1.t, log1.v, 'Color', [0.5 0.2 0.7], 'LineWidth', 1.5);
grid on;  xlabel('time (s)');  ylabel('speed (m/s)');
legend({'baseline', 'calibrated'});  title('Approach to the stop line');
subplot(1, 3, 3);
plot(1:nIter, bestHist, 'LineWidth', 1.5);  hold on;
plot(1:nIter, meanHist, '--', 'LineWidth', 1);
ylim([-40 0]);  grid on;  xlabel('iteration');  ylabel('reward');
legend({'best candidate', 'population mean'}, 'Location', 'southeast');
title('Learning curve');

% ----------------------------------------------------------------------------
function lut = lutFrom(theta, vMax)
% Monotonic table from non-negative steps (this is the LUT constraint).
lut = min(cumsum([0, abs(theta)]), vMax);
end

function [cost, lg] = rollout(bp, lut)
% Drive one approach from 40 m at 10 m/s with the given table.
dt = 0.02;  tau = 0.25;
d = 40;  v = 10;  a = 0;  t = 0;  J = 0;  aMin = 0;
nMax = round(40/dt);
lg.t = zeros(nMax, 1);  lg.v = lg.t;  lg.a = lg.t;  lg.d = lg.t;
k = 0;
while t < 40
    vt = interp1(bp, lut, min(max(d, 0), bp(end)));
    aN = min(max((vt - v)/tau, -8), 2);
    J  = J + ((aN - a)/dt)^2*dt;                  % jerk energy
    a  = aN;  aMin = min(aMin, a);
    v  = max(0, v + a*dt);
    d  = d - v*dt;
    t  = t + dt;
    k  = k + 1;
    lg.t(k) = t;  lg.v(k) = v;  lg.a(k) = a;  lg.d(k) = d;
    if (v < 0.05 && d < 1.0) || d < -3
        break
    end
end
lg.t = lg.t(1:k);  lg.v = lg.v(1:k);  lg.a = lg.a(1:k);  lg.d = lg.d(1:k);
over = max(0, -d);                                % went past the line
if v < 0.05
    short = max(0, d - 0.5);                      % stopped too early
else
    short = d + 5;                                % never stopped
end
cost = t + 0.002*J + 50*over + 5*short + 4*max(0, -aMin - 3.0)^2 + 20*(t >= 40);
end
