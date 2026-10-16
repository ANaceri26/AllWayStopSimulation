function R = run_demo(name, viewMode, gifFile)
%RUN_DEMO  Simulate and animate one all-way-stop scenario.
%   run_demo                       pick a scenario from a list
%   run_demo('queue')              2-D top view
%   run_demo('deadlock4', '3d')    3-D view
%   run_demo('runner', '3d', 'runner.gif')    also record a GIF
%
%   Scenarios: single, fifo, tie_right, head_on_left, concurrent, deadlock4,
%              queue, pedestrian, runner, false_start, rush_hour

if nargin < 2, viewMode = '2d'; end
if nargin < 3, gifFile  = '';   end
if nargin < 1 || isempty(name)
    names = aws_scenario('list');
    [sel, ok] = listdlg('ListString', names, 'SelectionMode', 'single', ...
        'PromptString', 'Choose a scenario', 'ListSize', [220 220]);
    if ~ok, return, end
    name = names{sel};
    btn = questdlg('Which view?', 'View', '2d', '3d', '2d');
    if ~isempty(btn), viewMode = btn; end
end

[cars, peds, P, expect, info] = aws_scenario(name);
fprintf('\nScenario "%s": %s\n', name, info);

tic;
R = aws_simulate(P, cars, peds);
R.name = name;
fprintf('Simulated %.1f s of traffic (%d cars) in %.2f s.\n', R.t(end), numel(cars), toc);

res = aws_check(R, expect);
fprintf('Verdict: %s\n', res.text);

aws_play(R, struct('view', viewMode, 'speed', 1, 'gif', gifFile));
aws_report(R);
end
