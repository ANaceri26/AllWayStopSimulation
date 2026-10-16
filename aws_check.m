function res = aws_check(R, expect)
%AWS_CHECK  Pass/fail verdict for one simulation run.
%   res = aws_check(R, expect)
%   Always checked (safety + liveness):
%     - no collision, ever
%     - no AV crosses its stop line before it has the right of way
%     - no two conflicting cars are inside the box together
%     - every car eventually clears the intersection
%   Optional fields of expect:
%     order       exact order in which the cars must enter (car IDs)
%     together    Kx2 pairs of cars that must be in the box at the same time
%     falseStart  exact number of false starts that must occur
%     minSep      smallest allowed gap to anything (m)
%     maxDelay    largest allowed delay of any car (s)
%     allowBoxConflict   true to tolerate brief box conflicts

if nargin < 2, expect = struct(); end
msgs = {};

if R.nCollisionSteps > 0
    msgs{end + 1} = sprintf('COLLISION (%d time steps, min gap %.2f m)', R.nCollisionSteps, R.minSep);
end
if R.nStopViolations > 0
    msgs{end + 1} = 'an AV crossed the stop line without right of way';
end
if R.nBoxConflictSteps > 0 && ~(isfield(expect, 'allowBoxConflict') && expect.allowBoxConflict)
    msgs{end + 1} = 'two conflicting cars were inside the intersection together';
end
if ~R.allCleared
    msgs{end + 1} = sprintf('%d car(s) never cleared (gridlock or Tend too short)', sum(isnan(R.tClr)));
end
av = ~R.isRunner;
if any(isnan(R.tArr(av)) & ~isnan(R.tClr(av)))
    msgs{end + 1} = 'an AV went through without a full stop';
end
if isfield(expect, 'order') && ~isequal(R.order(:)', expect.order(:)')
    msgs{end + 1} = sprintf('entry order was [%s], expected [%s]', ...
        num2str(R.order(:)'), num2str(expect.order(:)'));
end
if isfield(expect, 'together')
    for k = 1:size(expect.together, 1)
        a = expect.together(k, 1);  b = expect.together(k, 2);
        if ~(R.tGo(a) < R.tClr(b) && R.tGo(b) < R.tClr(a))
            msgs{end + 1} = sprintf('cars %d and %d should have crossed together', a, b); %#ok<AGROW>
        end
    end
end
if isfield(expect, 'falseStart') && R.nFalseStart ~= expect.falseStart
    msgs{end + 1} = sprintf('%d false start(s), expected %d', R.nFalseStart, expect.falseStart);
end
if isfield(expect, 'minSep') && R.minSep < expect.minSep
    msgs{end + 1} = sprintf('min gap %.2f m is below %.2f m', R.minSep, expect.minSep);
end
if isfield(expect, 'maxDelay') && max(R.delay) > expect.maxDelay
    msgs{end + 1} = sprintf('max delay %.1f s exceeds %.1f s', max(R.delay), expect.maxDelay);
end

res.pass = isempty(msgs);
res.msgs = msgs;
if res.pass
    res.text = 'PASS';
else
    res.text = ['FAIL: ' strjoin(msgs, '; ')];
end
end
