%RUN_ALL_TESTS  Headless regression suite for the all-way-stop simulator.
%   Runs every scenario (no graphics) and prints a PASS/FAIL table.
%     Part 1: scripted scenarios with V2X  -> exact expected behaviour
%     Part 2: same scenarios, sensors only (noisy arrival-time estimates)
%             -> must still be safe and must not gridlock
%     Part 3: random traffic, several seeds -> safety + liveness
%   Takes roughly a minute on a laptop.  For the MATLAB unit-test framework
%   version use:   results = runtests('AllWayStopTest')

names  = aws_scenario('list');
nPass  = 0;
nTotal = 0;
tAll   = tic;

fprintf('\n=== Part 1: scripted scenarios (as designed) ===========================\n');
fprintf('%-14s %-6s %7s %8s %7s  %s\n', 'scenario', 'result', 'cars', 'minGap', 'delay', 'entry order / reason');
for k = 1:numel(names)
    [cars, peds, P, expect] = aws_scenario(names{k});
    R   = aws_simulate(P, cars, peds);
    res = aws_check(R, expect);
    nTotal = nTotal + 1;  nPass = nPass + res.pass;
    if res.pass, note = ['[' num2str(R.order) ']']; else, note = strjoin(res.msgs, '; '); end
    if numel(note) > 60, note = [note(1:57) '...']; end
    fprintf('%-14s %-6s %7d %8.2f %7.1f  %s\n', names{k}, passStr(res.pass), numel(cars), ...
        R.minSep, mean(R.delay), note);
end

fprintf('\n=== Part 2: sensors only, noisy perception (sigma = 0.6 s) =============\n');
fprintf('%-14s %-6s %7s %8s %7s  %s\n', 'scenario', 'result', 'cars', 'minGap', 'delay', 'false starts / reason');
for k = 1:numel(names)
    [cars, peds, P] = aws_scenario(names{k});
    P.v2x = false;  P.sensorSigma = 0.6;  P.noiseOverride = [];
    R   = aws_simulate(P, cars, peds);
    res = aws_check(R, struct());            % safety + liveness only
    nTotal = nTotal + 1;  nPass = nPass + res.pass;
    if res.pass, note = sprintf('%d', R.nFalseStart); else, note = strjoin(res.msgs, '; '); end
    fprintf('%-14s %-6s %7d %8.2f %7.1f  %s\n', names{k}, passStr(res.pass), numel(cars), ...
        R.minSep, mean(R.delay), note);
end

fprintf('\n=== Part 3: random traffic ============================================\n');
fprintf('%-14s %-6s %7s %8s %7s  %s\n', 'seed / mode', 'result', 'cars', 'minGap', 'delay', 'false starts / reason');
for seed = 1:6
    for md = 1:2
        P = aws_params();
        P.seed = seed;  P.Tend = 240;
        if md == 2, P.v2x = false;  P.sensorSigma = 1.0; end
        rng(100 + seed);
        cars = aws_random_cars(0.08, 30, 0.25, 0.25);
        if isempty(cars), continue, end
        R   = aws_simulate(P, cars, []);
        res = aws_check(R, struct());
        nTotal = nTotal + 1;  nPass = nPass + res.pass;
        if res.pass, note = sprintf('%d', R.nFalseStart); else, note = strjoin(res.msgs, '; '); end
        if md == 1, tag = sprintf('%d  V2X', seed); else, tag = sprintf('%d  sensors', seed); end
        fprintf('%-14s %-6s %7d %8.2f %7.1f  %s\n', tag, passStr(res.pass), numel(cars), ...
            R.minSep, mean(R.delay), note);
    end
end

fprintf('\n%d of %d tests passed  (%.0f s)\n\n', nPass, nTotal, toc(tAll));
if nPass < nTotal
    warning('run_all_tests:failed', '%d test(s) FAILED - see the table above.', nTotal - nPass);
end

function s = passStr(tf)
if tf, s = 'PASS'; else, s = 'FAIL'; end
end
