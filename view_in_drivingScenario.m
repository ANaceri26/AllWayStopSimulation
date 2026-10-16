function view_in_drivingScenario(R, egoCar)
%VIEW_IN_DRIVINGSCENARIO  OPTIONAL: replay a run in Automated Driving Toolbox.
%   view_in_drivingScenario(R)          bird's-eye plot + chase camera on car 1
%   view_in_drivingScenario(R, 3)       chase camera follows car 3
%
%   Needs Automated Driving Toolbox (everything else in this folder does not).
%   It only replays the trajectories computed by aws_simulate, so it is light
%   enough for integrated graphics.  This is the closest thing to a RoadRunner
%   style view without RoadRunner.
%
%   NOTE: this file is the one part of the project that was written without
%   being exercised; if your release complains, send me the error message.
%
%   Example:
%       [cars, peds, P] = aws_scenario('queue');
%       R = aws_simulate(P, cars, peds);
%       view_in_drivingScenario(R, 2)

if nargin < 2, egoCar = 1; end
if isempty(which('drivingScenario'))
    error('view_in_drivingScenario:noADT', ...
        'Automated Driving Toolbox is not installed. Use aws_play(R, struct(''view'', ''3d'')) instead.');
end

P = R.P;
N = size(R.X, 2);
scn = drivingScenario('SampleTime', P.dt);
road(scn, [0 -P.L 0; 0 P.L 0], 'Lanes', lanespec(2, 'Width', P.w));
road(scn, [-P.L 0 0; P.L 0 0], 'Lanes', lanespec(2, 'Width', P.w));

parked = @(i) [500 + 10*i, 500, 0];           % where cars wait off-stage
veh = cell(1, N);
for i = 1:N
    veh{i} = vehicle(scn, 'ClassID', 1, 'Length', P.carLen, 'Width', P.carWid, ...
        'Position', parked(i));
end

plot(scn);
chasePlot(veh{egoCar});

for k = 1:2:numel(R.t)
    for i = 1:N
        if isnan(R.X(k, i))
            veh{i}.Position = parked(i);
        else
            % vehicle position refers to the rear axle: shift back from centre
            yaw = R.YAW(k, i);
            veh{i}.Position = [R.X(k, i) - 1.4*cos(yaw), R.Y(k, i) - 1.4*sin(yaw), 0];
            veh{i}.Yaw      = yaw*180/pi;
        end
    end
    updatePlots(scn);
    drawnow limitrate;
    pause(P.dt);
end
end
