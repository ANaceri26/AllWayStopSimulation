%DEMO_SENSOR_FUSION  Uncertainty-aware fusion with Dempster-Shafer theory.
%   Idea from the "Uncertainty-Based Sensor Fusion" talk, applied to the stop
%   sign problem: my AV is waiting at the line and must decide whether the car
%   on the cross street is YIELDING (will stop) or GOING (will run the sign).
%
%   Three sources give evidence, each with its own reliability:
%     radar   - is the car decelerating enough?        (reliable)
%     camera  - are the brake lights on?               (blinded by glare 1.5-2.5 s)
%     V2X     - the car's own "I will stop" message    (can be wrong / spoofed)
%
%   Each source gives a mass vector [m(Yield) m(Go) m(Unknown)].  "Unknown" is
%   what makes this different from plain Bayes: an unreliable sensor says
%   "I don't know" instead of a confident wrong number.  Sources are combined
%   with Dempster's rule; K is the conflict between them.
%   Base MATLAB only.

rng(3);
t  = (0:0.1:4)';
nT = numel(t);
caseNames = {'Case A: car really stops', 'Case B: car runs the sign, V2X message says "stopping"'};

figure('Color', 'w', 'Name', 'Dempster-Shafer fusion', 'NumberTitle', 'off');
for c = 1:2
    % --- ground truth of the other car -------------------------------------
    if c == 1
        decel = 2.5*ones(nT, 1);         % braking normally
        brakeLight = true(nT, 1);
    else
        decel = 0.1*ones(nT, 1);         % not braking at all
        brakeLight = false(nT, 1);
    end

    bel = zeros(nT, 1);  pl = zeros(nT, 1);  K = zeros(nT, 1);  bayes = zeros(nT, 1);
    for k = 1:nT
        % --- radar ---------------------------------------------------------
        aMeas = decel(k) + 0.4*randn;
        pR    = 1/(1 + exp(-(aMeas - 1.2)/0.4));
        mR    = massOf(pR, 0.85);
        % --- camera (glare window) -----------------------------------------
        seen = brakeLight(k);
        if rand < 0.1, seen = ~seen; end                 % detection errors
        if seen, pC = 0.9; else, pC = 0.15; end
        if t(k) >= 1.5 && t(k) <= 2.5
            relC = 0.05;  pC = rand;                     % blinded: garbage in
        else
            relC = 0.6;
        end
        mC = massOf(pC, relC);
        % --- V2X intent message (always claims "I will stop") --------------
        pV = 0.98;
        mV = massOf(pV, 0.7);

        % --- Dempster-Shafer combination -----------------------------------
        [m12, k1] = dsCombine(mR, mC);
        [m,   k2] = dsCombine(m12, mV);
        bel(k) = m(1);                   % belief:       evidence FOR yielding
        pl(k)  = m(1) + m(3);            % plausibility: 1 - evidence AGAINST
        K(k)   = 1 - (1 - k1)*(1 - k2);  % total conflict

        % --- naive Bayes for comparison (no notion of reliability) ---------
        num = pR*pC*pV;
        bayes(k) = num/(num + (1 - pR)*(1 - pC)*(1 - pV));
    end
    go = bel > 0.85 & K < 0.4;           % enter only if sure AND sources agree

    subplot(3, 2, c);
    hold on;
    fill([t; flipud(t)], [bel; flipud(pl)], [0.6 0.8 1], 'EdgeColor', 'none');
    plot(t, bel, 'b-', 'LineWidth', 1.5);
    plot(t, bayes, 'k--', 'LineWidth', 1);
    ylim([0 1.05]);  grid on;
    title(caseNames{c});  ylabel('P(yielding)');
    legend({'uncertainty band (Bel..Pl)', 'belief', 'naive Bayes'}, 'Location', 'best');

    subplot(3, 2, 2 + c);
    plot(t, K, 'r-', 'LineWidth', 1.5);
    ylim([0 1]);  grid on;  ylabel('conflict K');
    title('disagreement between the sources');

    subplot(3, 2, 4 + c);
    stairs(t, double(go), 'LineWidth', 1.5, 'Color', [0.1 0.6 0.2]);
    ylim([-0.1 1.1]);  grid on;  xlabel('time (s)');
    set(gca, 'YTick', [0 1], 'YTickLabel', {'HOLD', 'GO'});
    title('my decision');

    fprintf('%s\n   mean belief %.2f | mean conflict %.2f | naive Bayes %.2f | GO for %.0f%% of the time\n', ...
        caseNames{c}, mean(bel), mean(K), mean(bayes), 100*mean(go));
end
fprintf(['\nIn case B the wrong V2X message drags naive Bayes toward "yielding" and it has no\n' ...
         'way to say that its sources disagree.  Dempster-Shafer reports a high conflict K,\n' ...
         'so the car keeps holding.\n']);

% ----------------------------------------------------------------------------
function m = massOf(p, reliability)
% Turn a probability and a reliability into masses [Yield, Go, Unknown].
m = [reliability*p, reliability*(1 - p), 1 - reliability];
end

function [m, K] = dsCombine(m1, m2)
% Dempster's rule for the frame {Yield, Go}; index 3 is the whole frame.
K = m1(1)*m2(2) + m1(2)*m2(1);                      % mass on the empty set
m = [m1(1)*m2(1) + m1(1)*m2(3) + m1(3)*m2(1), ...
     m1(2)*m2(2) + m1(2)*m2(3) + m1(3)*m2(2), ...
     m1(3)*m2(3)]/(1 - K);
end
