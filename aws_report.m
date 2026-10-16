function aws_report(R)
%AWS_REPORT  Summary figure for one run: speed traces and a who-did-what timeline.

N = size(R.X, 2);
dirs = 'SENW';
stateCol = [0.20 0.45 0.90; 1.00 0.70 0.10; 0.20 0.75 0.30; 0.62 0.62 0.67];

figure('Color', 'w', 'Name', 'Run report', 'NumberTitle', 'off');

subplot(2, 1, 1);
plot(R.t, R.V*3.6, 'LineWidth', 1.2);
grid on;
xlabel('time (s)');  ylabel('speed (km/h)');
title(sprintf('%s: speed of every car', strrep(R.name, '_', ' ')));

subplot(2, 1, 2);
hold on;
labels = cell(1, N);
for i = 1:N
    for stt = 1:4
        on = (R.ST(:, i) == stt);
        if ~any(on), continue, end
        d   = diff([0; on; 0]);
        k0  = find(d == 1);
        k1  = find(d == -1) - 1;
        for q = 1:numel(k0)
            line([R.t(k0(q)) R.t(k1(q))], [i i], 'Color', stateCol(stt, :), 'LineWidth', 7);
        end
    end
    labels{i} = sprintf('car %d  %s-%s', i, dirs(R.cars(i).approach + 1), R.cars(i).turn);
    if R.isRunner(i), labels{i} = [labels{i} ' (human)']; end
end
set(gca, 'YTick', 1:N, 'YTickLabel', labels, 'YDir', 'reverse', 'YLim', [0.3 N + 0.7]);
grid on;
xlabel('time (s)');
title('timeline:  blue = approaching   amber = waiting at the line   green = crossing   grey = cleared');

fprintf('\n  %-22s %8s %8s %8s %8s\n', 'car', 'stopped', 'entered', 'cleared', 'delay');
for i = 1:N
    fprintf('  %-22s %8.2f %8.2f %8.2f %8.2f\n', labels{i}, R.tArr(i), R.tGo(i), R.tClr(i), R.delay(i));
end
fprintf('  entry order: [%s]\n', num2str(R.order));
fprintf('  min gap to anything: %.2f m   collisions: %d   false starts: %d\n\n', ...
    R.minSep, R.nCollisionSteps > 0, R.nFalseStart);
end
