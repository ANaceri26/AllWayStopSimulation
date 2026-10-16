%DEMO_NN_SURROGATE  Replace the slow simulation with a fast neural network.
%   Idea from the GM talk "Harnessing Neural Networks to Enable Fast
%   Vehicle-Level Simulations": run the detailed model over its operating
%   range once, train a small network on the results, then use the network
%   wherever speed matters.
%
%   Here the "slow model" is the intersection simulator and the network learns
%       [arrival rate, share of left turns, share of right turns] -> mean delay
%   Traffic arrives at steady headways here (not randomly) so that each
%   operating point has one repeatable answer, like GM's steady-state points.
%   The network (3-8-8-1, tanh) and its Adam training loop are written out
%   in plain MATLAB, so no toolbox is needed.
%
%   The first run builds the training data (several minutes) and saves it to
%   aws_nn_data.mat; later runs load it and finish in seconds.

nSamples = 150;
dataFile = 'aws_nn_data.mat';

% --- 1) training data from the full simulation ------------------------------
if exist(dataFile, 'file')
    S = load(dataFile);
    Xd = S.Xd;  Yd = S.Yd;  tPerSim = S.tPerSim;
    fprintf('Loaded %d samples from %s (delete the file to regenerate).\n', size(Xd, 1), dataFile);
else
    rng(42);
    Xd = [0.02 + 0.10*rand(nSamples, 1), 0.4*rand(nSamples, 1), 0.4*rand(nSamples, 1)];
    Yd = nan(nSamples, 1);
    tStart = tic;
    for n = 1:nSamples
        P = aws_params();
        P.Tend = 500;
        cars = steadyCars(Xd(n, 1), 60, Xd(n, 2), Xd(n, 3));
        R = aws_simulate(P, cars, []);
        Yd(n) = mean(R.delay(~isnan(R.delay)));
        if mod(n, 10) == 0
            fprintf('  simulated %3d / %d   (%.0f s elapsed)\n', n, nSamples, toc(tStart));
        end
    end
    tPerSim = toc(tStart)/nSamples;
    ok = ~isnan(Yd);
    Xd = Xd(ok, :);  Yd = Yd(ok);
    save(dataFile, 'Xd', 'Yd', 'tPerSim');
end

% --- 2) normalise and split -------------------------------------------------
n   = size(Xd, 1);
rng(1);
idx = randperm(n);
nTr = round(0.8*n);
tr  = idx(1:nTr);  te = idx(nTr + 1:end);
muX = mean(Xd(tr, :), 1);  sdX = std(Xd(tr, :), 0, 1);
muY = mean(Yd(tr));        sdY = std(Yd(tr));
Xn  = (Xd - muX)./sdX;
Yn  = (Yd - muY)/sdY;

% --- 3) train ---------------------------------------------------------------
[net, loss] = mlpTrain(Xn(tr, :), Yn(tr), [8 8], 1000, 0.01, 0.003);

predTr = mlpPredict(net, Xn(tr, :))*sdY + muY;
predTe = mlpPredict(net, Xn(te, :))*sdY + muY;
r2 = @(y, p) 1 - sum((y - p).^2)/sum((y - mean(y)).^2);
fprintf('R^2  train = %.3f   test = %.3f   (test RMSE %.1f s)\n', ...
    r2(Yd(tr), predTr), r2(Yd(te), predTe), sqrt(mean((Yd(te) - predTe).^2)));

% --- 4) use it: a full operating map in a blink ----------------------------
[RR, LL] = meshgrid(linspace(0.02, 0.12, 60), linspace(0, 0.4, 60));
Q  = [RR(:), LL(:), 0.2*ones(numel(RR), 1)];
tic;
D  = mlpPredict(net, (Q - muX)./sdX)*sdY + muY;
tNN = toc/size(Q, 1);
fprintf('Simulation: %.2f s per operating point.  Network: %.1e s.  Speed-up: about %.0fx\n', ...
    tPerSim, tNN, tPerSim/tNN);

figure('Color', 'w', 'Name', 'NN surrogate', 'NumberTitle', 'off');
subplot(1, 3, 1);
semilogy(loss, 'LineWidth', 1.2);  grid on;
xlabel('epoch');  ylabel('training loss (MSE, normalised)');  title('Training');
subplot(1, 3, 2);
plot(Yd(tr), predTr, 'b.', Yd(te), predTe, 'ro');  hold on;
lim = [0, max(Yd)*1.05];  plot(lim, lim, 'k--');  grid on;  axis([lim lim]);
xlabel('simulated delay (s)');  ylabel('network prediction (s)');
legend({'train', 'test'}, 'Location', 'northwest');  title('Surrogate accuracy');
subplot(1, 3, 3);
surf(RR*3600, LL*100, reshape(D, size(RR)), 'EdgeColor', 'none');
xlabel('arrivals per approach (veh/h)');  ylabel('left turns (%)');  zlabel('mean delay (s)');
title('Delay map from the network (3600 points)');  view(-40, 30);

% ----------------------------------------------------------------------------
function [net, loss] = mlpTrain(X, Y, hidden, nEpoch, lr, wd)
% Small fully connected network trained with full-batch Adam.
% wd = weight decay (keeps the small data set from being memorised).
sizes = [size(X, 2), hidden, 1];
nL = numel(sizes) - 1;
n  = size(X, 1);
W = cell(1, nL);  B = cell(1, nL);
mW = cell(1, nL); vW = cell(1, nL); mB = cell(1, nL); vB = cell(1, nL);
for l = 1:nL
    W{l}  = randn(sizes(l + 1), sizes(l))*sqrt(1/sizes(l));
    B{l}  = zeros(sizes(l + 1), 1);
    mW{l} = zeros(size(W{l}));  vW{l} = mW{l};
    mB{l} = zeros(size(B{l}));  vB{l} = mB{l};
end
b1 = 0.9;  b2 = 0.999;  ep = 1e-8;
loss = zeros(nEpoch, 1);
A = cell(1, nL + 1);
for it = 1:nEpoch
    A{1} = X';                                   % forward pass
    for l = 1:nL
        Z = W{l}*A{l} + repmat(B{l}, 1, n);
        if l < nL, A{l + 1} = tanh(Z); else, A{l + 1} = Z; end
    end
    E = A{nL + 1} - Y';
    loss(it) = mean(E.^2);
    D = 2*E/n;                                   % backward pass
    for l = nL:-1:1
        gW = D*A{l}' + 2*wd*W{l};
        gB = sum(D, 2);
        if l > 1
            D = (W{l}'*D).*(1 - A{l}.^2);
        end
        mW{l} = b1*mW{l} + (1 - b1)*gW;   vW{l} = b2*vW{l} + (1 - b2)*gW.^2;
        mB{l} = b1*mB{l} + (1 - b1)*gB;   vB{l} = b2*vB{l} + (1 - b2)*gB.^2;
        c1 = 1 - b1^it;  c2 = 1 - b2^it;
        W{l} = W{l} - lr*(mW{l}/c1)./(sqrt(vW{l}/c2) + ep);
        B{l} = B{l} - lr*(mB{l}/c1)./(sqrt(vB{l}/c2) + ep);
    end
end
net.W = W;  net.B = B;
end

function Y = mlpPredict(net, X)
A  = X';
nL = numel(net.W);
for l = 1:nL
    Z = net.W{l}*A + repmat(net.B{l}, 1, size(A, 2));
    if l < nL, A = tanh(Z); else, A = Z; end
end
Y = A';
end

function cars = steadyCars(rate, T, pLeft, pRight)
% One car every 1/rate seconds on each approach (approaches offset by a
% quarter headway), turns handed out by a fixed pattern.
cars = struct('approach', {}, 'turn', {}, 'tSpawn', {}, 'dStart', {}, 'type', {});
k = 0;
for a = 0:3
    t = a*0.25/rate;
    while t < T
        k = k + 1;
        u = mod(k*0.6180339887, 1);
        if u < pLeft
            turn = 'L';
        elseif u < pLeft + pRight
            turn = 'R';
        else
            turn = 'S';
        end
        cars(k).approach = a;
        cars(k).turn     = turn;
        cars(k).tSpawn   = t;
        cars(k).dStart   = 40;
        cars(k).type     = 'av';
        t = t + 1/rate;
    end
end
[~, idx] = sort([cars.tSpawn]);
cars = cars(idx);
end
