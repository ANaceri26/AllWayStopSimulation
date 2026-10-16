%RUN_MONTECARLO  How do traffic volume and perception quality change delay?
%   Sweeps the arrival rate and compares three "architectures":
%     1) V2X          cars share exact stop times (distributed, service based)
%     2) sensors      each car estimates the others, small error (0.5 s)
%     3) poor sensors large estimation error (1.5 s)
%   48 short simulations - expect a few minutes on a laptop.

rates  = [0.03 0.06 0.09 0.12];          % arrivals per second per approach
sigmas = [0 0.5 1.5];
useV2X = [true false false];
labels = {'V2X', 'sensors, \sigma = 0.5 s', 'sensors, \sigma = 1.5 s'};
nSeeds = 4;
Tarr   = 40;

delay  = nan(numel(rates), 3);
falseS = zeros(numel(rates), 3);
minGap = inf(numel(rates), 3);
unsafe = 0;
tAll   = tic;
for r = 1:numel(rates)
    for m = 1:3
        d = [];
        for seed = 1:nSeeds
            P = aws_params();
            P.seed = seed;  P.Tend = 300;
            P.v2x  = useV2X(m);  P.sensorSigma = sigmas(m);
            rng(1000*r + seed);                 % same traffic for all three modes
            cars = aws_random_cars(rates(r), Tarr, 0.2, 0.2);
            if isempty(cars), continue, end
            R = aws_simulate(P, cars, []);
            d = [d, R.delay];                   %#ok<AGROW>
            falseS(r, m) = falseS(r, m) + R.nFalseStart;
            minGap(r, m) = min(minGap(r, m), R.minSep);
            unsafe = unsafe + (R.nCollisionSteps > 0) + ~R.allCleared;
        end
        delay(r, m) = mean(d(~isnan(d)));
        fprintf('rate %4.0f veh/h per approach | %-24s | mean delay %5.1f s | false starts %d\n', ...
            rates(r)*3600, strrep(labels{m}, '\sigma', 'sigma'), delay(r, m), falseS(r, m));
    end
end
fprintf('Done in %.0f s.  Unsafe or gridlocked runs: %d (should be 0)\n', toc(tAll), unsafe);

figure('Color', 'w', 'Name', 'Monte Carlo', 'NumberTitle', 'off');
subplot(1, 2, 1);
plot(rates*3600, delay, '-o', 'LineWidth', 1.5);
grid on;  xlabel('arrivals per approach (veh/h)');  ylabel('mean delay per car (s)');
title('Delay vs. traffic volume');  legend(labels, 'Location', 'northwest');
subplot(1, 2, 2);
bar(rates*3600, falseS);
grid on;  xlabel('arrivals per approach (veh/h)');  ylabel('false starts (all seeds)');
title('Right-of-way misunderstandings');  legend(labels, 'Location', 'northwest');
