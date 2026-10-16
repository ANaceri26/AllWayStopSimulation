function R = aws_simulate(P, cars, peds)
%AWS_SIMULATE  Simulate autonomous cars negotiating an all-way-stop intersection.
%   R = aws_simulate(P, cars, peds)
%
%   P     parameters from aws_params
%   cars  struct array with fields
%           approach  0=from South, 1=from East, 2=from North, 3=from West
%           turn      'S' | 'L' | 'R'
%           tSpawn    time the car appears (s)
%           dStart    distance from the stop line when it appears (m)
%           type      'av' (obeys the rules) | 'runner' (human who runs the sign)
%   peds  (optional) struct array with fields p0, p1 (1x2), t0, speed
%
%   Each AV runs the same decision logic every time step:
%     1. come to a full stop at the line and wait P.dwell seconds
%     2. never enter while a conflicting car is in the box
%     3. never enter if a conflicting car is approaching too fast to stop
%     4. never enter if a pedestrian is (or will soon be) on my path
%     5. first to stop goes first; ties -> car on the right goes first;
%        head-on ties -> straight/right before left
%     6. if everybody is waiting for everybody (4-way tie) a patience timer
%        breaks the deadlock (ordered token with V2X, random without)
%   On top of that every car has an "AEB" layer that brakes for anything
%   found on its path ahead (lead car, pedestrian, rule breaker).
%
%   R contains the logged trajectories and safety / performance metrics.

if nargin < 3 || isempty(peds)
    peds = struct('p0', {}, 'p1', {}, 't0', {}, 'speed', {});
end

NOTYET = 0; APPROACH = 1; WAIT = 2; GO = 3; EXITED = 4; DONE = 5; %#ok<NASGU>

rng(P.seed);
N   = numel(cars);
M   = numel(peds);
dt  = P.dt;
hl  = P.carLen/2;
hw  = P.carWid/2;

% --- the 12 possible movements and their conflict table --------------------
turnChars = 'SLR';
mpaths = cell(1, 12);
for a = 0:3
    for q = 1:3
        mpaths{a*3 + q} = aws_path(P, a, turnChars(q));
    end
end
Cm = false(12);
for m1 = 1:12
    for m2 = m1 + 1:12
        if ceil(m1/3) == ceil(m2/3)
            continue                       % same approach: plain car following
        end
        A = mpaths{m1}.pts(mpaths{m1}.iLine:mpaths{m1}.iExit + 20, :);
        B = mpaths{m2}.pts(mpaths{m2}.iLine:mpaths{m2}.iExit + 20, :);
        dx = bsxfun(@minus, A(:, 1), B(:, 1)');
        dy = bsxfun(@minus, A(:, 2), B(:, 2)');
        Cm(m1, m2) = min(min(hypot(dx, dy))) < P.conflictDist;
        Cm(m2, m1) = Cm(m1, m2);
    end
end

appr     = zeros(1, N);
mov      = zeros(1, N);
isLeft   = false(1, N);
isRunner = false(1, N);
tSpawn   = zeros(1, N);
dStart   = zeros(1, N);
for i = 1:N
    appr(i)     = cars(i).approach;
    q           = find(turnChars == upper(cars(i).turn), 1);
    mov(i)      = appr(i)*3 + q;
    isLeft(i)   = (q == 2);
    isRunner(i) = strcmpi(cars(i).type, 'runner');
    tSpawn(i)   = cars(i).tSpawn;
    dStart(i)   = cars(i).dStart;
end
C = Cm(mov, mov);                          % NxN conflict matrix between cars
sLine = zeros(1, N);  sExit = zeros(1, N);  sEnd = zeros(1, N);  Rad = zeros(1, N);
for i = 1:N
    sLine(i) = mpaths{mov(i)}.sLine;
    sExit(i) = mpaths{mov(i)}.sExit;
    sEnd(i)  = mpaths{mov(i)}.s(end);
    Rad(i)   = mpaths{mov(i)}.R;
end

% --- state -----------------------------------------------------------------
st   = zeros(1, N);
s    = zeros(1, N);
v    = zeros(1, N);
tArr = nan(1, N);  tGo = nan(1, N);  tClr = nan(1, N);
blockedSince = nan(1, N);
aborted      = false(1, N);
noise        = zeros(N, N);                % noise(i,j): error of car i about car j
if P.v2x
    patience = 0.5 + 0.2*(0:N - 1);        % deterministic token order
else
    patience = 1.0 + 2.0*rand(1, N);       % random back-off
end

K   = round(P.Tend/dt);
X   = nan(K, N);  Y = nan(K, N);  YAW = nan(K, N);  V = nan(K, N);
ST  = zeros(K, N);
PX  = nan(K, max(M, 1));  PY = nan(K, max(M, 1));

nColl = 0;  minSep = inf;  nFalse = 0;  nBoxConf = 0;  nViol = 0;
kEnd  = K;
nLook = round(P.lookAhead/P.ds);
xs = zeros(1, N);  ys = zeros(1, N);  yw = zeros(1, N);

for k = 1:K
    t = (k - 1)*dt;

    % ---- pedestrians ------------------------------------------------------
    pp = zeros(0, 2);  pv = zeros(0, 2);
    for q = 1:M
        p0 = peds(q).p0(:)';  p1 = peds(q).p1(:)';
        D  = norm(p1 - p0);
        u  = (p1 - p0)/D;
        trav = (t - peds(q).t0)*peds(q).speed;
        if trav >= 0 && trav <= D
            pos = p0 + u*trav;
            pp(end + 1, :) = pos;                   %#ok<AGROW>
            pv(end + 1, :) = u*peds(q).speed;       %#ok<AGROW>
            PX(k, q) = pos(1);  PY(k, q) = pos(2);
        end
    end

    % ---- spawn ------------------------------------------------------------
    for i = 1:N
        if st(i) == NOTYET && t >= tSpawn(i)
            sNew = max(sLine(i) - dStart(i) - hl, 0.6);
            gapS = inf;
            for j = 1:N
                if j ~= i && appr(j) == appr(i) && (st(j) == APPROACH || st(j) == WAIT)
                    gapS = min(gapS, s(j) - sNew - 2*hl);
                end
            end
            if gapS >= 4.0                           % room to appear safely
                st(i) = APPROACH;
                s(i)  = sNew;
                v(i)  = min(P.vCruise, sqrt(2*P.bComf*max(gapS - P.s0, 0)));
            end
        end
    end
    front = s + hl;

    % ---- stop-sign decisions (all cars decide on the same snapshot) -------
    grant   = false(1, N);
    anyGo   = any(st == GO);
    for i = 1:N
        if st(i) ~= WAIT || (t - tArr(i)) < P.dwell
            continue
        end
        hard = false;                       % physically unsafe to enter
        soft = false;                       % somebody else has priority
        for j = 1:N
            if j == i || ~C(i, j)
                continue
            end
            if st(j) == GO
                hard = true;                % rule 2
            elseif st(j) == APPROACH
                dj = sLine(j) - front(j);   % rule 3: can it still stop?
                if v(j) > 1 && v(j)^2/(2*max(dj, 0.1)) > 4.5 && dj/v(j) < 6
                    hard = true;
                end
            elseif st(j) == WAIT            % rule 5
                ti = tArr(i);
                tj = tArr(j) + noise(i, j);
                if tj < ti - P.tieTol
                    soft = true;                            % j stopped first
                elseif tj <= ti + P.tieTol                  % tie
                    if appr(j) == mod(appr(i) + 1, 4)
                        soft = true;                        % j is on my right
                    elseif appr(i) == mod(appr(j) + 1, 4)
                        % I am on j's right: I have priority
                    else
                        if isLeft(i) && ~isLeft(j)
                            soft = true;                    % left yields
                        elseif isLeft(i) == isLeft(j) && j < i
                            soft = true;                    % last resort: ID
                        end
                    end
                end
            end
        end
        if ~isempty(pp)                     % rule 4
            pth = mpaths{mov(i)};
            bx  = pth.pts(pth.iLine:pth.iExit + 12, :);
            for q = 1:size(pp, 1)
                for h = 0:4                 % now and the next 4 seconds
                    qq = pp(q, :) + h*pv(q, :);
                    if min(hypot(bx(:, 1) - qq(1), bx(:, 2) - qq(2))) < 2.5
                        hard = true;
                    end
                end
            end
        end
        if hard || anyGo
            blockedSince(i) = NaN;          % traffic is flowing: no deadlock
        end
        if hard
            continue
        end
        if soft                             % rule 6
            if isnan(blockedSince(i))
                blockedSince(i) = t;
            end
            if (t - blockedSince(i)) < patience(i)
                continue
            end
        end
        grant(i) = true;
    end
    for i = find(grant)
        st(i) = GO;
        if isnan(tGo(i)), tGo(i) = t; end
        blockedSince(i) = NaN;
    end

    % ---- false start: two conflicting cars went at once -> one backs off --
    for i = 1:N
        if st(i) == GO && ~isRunner(i)
            pI = front(i) - sLine(i);
            if pI < P.abortDist
                for j = 1:N
                    if j ~= i && C(i, j) && st(j) == GO
                        pJ = front(j) - sLine(j);
                        if pJ > pI + 1e-9 || (abs(pJ - pI) <= 1e-9 && j < i)
                            st(i)      = WAIT;
                            tGo(i)     = NaN;
                            aborted(i) = true;
                            nFalse     = nFalse + 1;
                            break
                        end
                    end
                end
            end
        end
    end

    % ---- poses ------------------------------------------------------------
    act = find(st >= APPROACH & st <= EXITED);
    for i = act
        [xs(i), ys(i), yw(i)] = localPose(mpaths{mov(i)}, s(i));
    end

    % ---- longitudinal control --------------------------------------------
    acc = zeros(1, N);
    for i = act
        pth = mpaths{mov(i)};
        vt  = P.vCruise;
        if isfinite(Rad(i)) && (sLine(i) - 2) < front(i) && (s(i) - hl) < sExit(i)
            vt = min(vt, sqrt(P.aLat*Rad(i)));          % slow down in the turn
        end
        if (st(i) == APPROACH || st(i) == WAIT) && ~isRunner(i)
            d  = sLine(i) - P.stopMargin - front(i);    % stop at the line
            vt = min(vt, sqrt(2*P.bComf*max(d, 0)));
        end
        % "AEB" layer: anything on my path ahead?
        i0 = find(pth.s >= front(i), 1);
        if ~isempty(i0)
            i1  = min(numel(pth.s), i0 + nLook - 1);
            seg = pth.pts(i0:2:i1, :);
            ss  = pth.s(i0:2:i1);
            for j = act
                if j == i, continue, end
                hx = cos(yw(j));  hy = sin(yw(j));
                gap = inf;
                for o = [-hl, 0, hl]                    % rear, centre, front of j
                    dd = hypot(seg(:, 1) - (xs(j) + o*hx), seg(:, 2) - (ys(j) + o*hy));
                    m  = find(dd < P.obsDist, 1);
                    if ~isempty(m)
                        gap = min(gap, ss(m) - front(i));
                    end
                end
                if isfinite(gap)
                    dotp = cos(yw(i) - yw(j));
                    if dotp > 0.5, vl = v(j)*dotp; else, vl = 0; end
                    if gap > P.s0
                        vt = min(vt, sqrt(2*P.bComf*(gap - P.s0) + vl^2));
                    else
                        vt = min(vt, vl*max(gap, 0)/P.s0);
                    end
                end
            end
            for q = 1:size(pp, 1)
                dd = hypot(seg(:, 1) - pp(q, 1), seg(:, 2) - pp(q, 2));
                m  = find(dd < 2.0, 1);
                if ~isempty(m)
                    gap = ss(m) - front(i);
                    vt  = min(vt, sqrt(2*P.bComf*max(gap - 1.0, 0)));
                end
            end
        end
        acc(i) = min(max((vt - v(i))/P.tau, -P.bMax), P.aMax);
    end
    for i = act
        v(i) = max(0, v(i) + acc(i)*dt);
        s(i) = s(i) + v(i)*dt;
    end
    front = s + hl;

    % ---- state transitions -----------------------------------------------
    for i = act
        switch st(i)
            case APPROACH
                if isRunner(i)
                    if front(i) > sLine(i)
                        st(i) = GO;  tGo(i) = t;
                    end
                elseif v(i) < 0.1 && (sLine(i) - front(i)) < 1.5
                    st(i)   = WAIT;                     % full stop at the line
                    tArr(i) = t;
                    if ~P.v2x
                        noise(:, i) = P.sensorSigma*randn(N, 1);
                    end
                    if ~isempty(P.noiseOverride)
                        noise(:, i) = P.noiseOverride(:, i);
                    end
                    noise(i, i) = 0;
                end
            case GO
                if (s(i) - hl) > sExit(i)
                    st(i) = EXITED;  tClr(i) = t;
                end
            case EXITED
                if s(i) > sEnd(i) - 3
                    st(i) = DONE;
                end
        end
    end

    % ---- logging and safety monitors -------------------------------------
    act = find(st >= APPROACH & st <= EXITED);
    for i = act
        [xs(i), ys(i), yw(i)] = localPose(mpaths{mov(i)}, s(i));
        X(k, i) = xs(i);  Y(k, i) = ys(i);  YAW(k, i) = yw(i);  V(k, i) = v(i);
        if ~isRunner(i) && ~aborted(i) && isnan(tGo(i)) && front(i) > sLine(i) + 0.3
            nViol = nViol + 1;              % crossed the line without the right to
        end
    end
    ST(k, :) = st;
    na = numel(act);
    for a = 1:na
        i = act(a);
        for b = a + 1:na
            j = act(b);
            if abs(xs(i) - xs(j)) + abs(ys(i) - ys(j)) < 14
                sep    = localRectSep(xs(i), ys(i), yw(i), hl, hw, xs(j), ys(j), yw(j), hl, hw);
                minSep = min(minSep, sep);
                if sep < 0, nColl = nColl + 1; end
            end
            if C(i, j) && st(i) == GO && st(j) == GO
                nBoxConf = nBoxConf + 1;
            end
        end
        for q = 1:size(pp, 1)
            sep    = localRectSep(xs(i), ys(i), yw(i), hl, hw, pp(q, 1), pp(q, 2), 0, 0.3, 0.3);
            minSep = min(minSep, sep);
            if sep < 0, nColl = nColl + 1; end
        end
    end

    if all(st == DONE) || (all(st >= EXITED) && t > max(tClr) + 3)
        kEnd = k;
        break
    end
end

% --- results ---------------------------------------------------------------
idx = 1:kEnd;
R.t   = (idx - 1)'*dt;
R.X   = X(idx, :);    R.Y  = Y(idx, :);   R.YAW = YAW(idx, :);
R.V   = V(idx, :);    R.ST = ST(idx, :);
R.PX  = PX(idx, :);   R.PY = PY(idx, :);
R.tArr = tArr;  R.tGo = tGo;  R.tClr = tClr;
tFree   = tSpawn + (sExit + hl - max(sLine - dStart - hl, 0.6))/P.vCruise;
R.delay = tClr - tFree;                    % time lost vs. driving straight through
[~, ord] = sort(tGo);
R.order  = ord(~isnan(tGo(ord)));          % car IDs in the order they entered
R.nCollisionSteps = nColl;
R.minSep     = minSep;
R.nFalseStart = nFalse;
R.nBoxConflictSteps = nBoxConf;
R.nStopViolations   = nViol;
R.allCleared = all(~isnan(tClr));
R.C     = C;
R.cars  = cars;
R.peds  = peds;
R.P     = P;
R.isRunner = isRunner;
R.name  = '';
end

% ============================================================================
function [x, y, yaw] = localPose(pth, s)
sq  = min(max([s - 0.5; s; s + 0.5], 0), pth.s(end));
xy  = interp1(pth.s, pth.pts, sq);
x   = xy(2, 1);
y   = xy(2, 2);
yaw = atan2(xy(3, 2) - xy(1, 2), xy(3, 1) - xy(1, 1));
end

function sep = localRectSep(xa, ya, ta, la, wa, xb, yb, tb, lb, wb)
% Separating-axis test for two oriented rectangles.
% sep > 0: lower bound of the gap between them;  sep < 0: they overlap.
ua = [cos(ta) sin(ta)];  na = [-sin(ta) cos(ta)];
ub = [cos(tb) sin(tb)];  nb = [-sin(tb) cos(tb)];
d  = [xb - xa, yb - ya];
ax = [ua; na; ub; nb];
sep = -inf;
for q = 1:4
    a  = ax(q, :);
    ra = la*abs(a*ua') + wa*abs(a*na');
    rb = lb*abs(a*ub') + wb*abs(a*nb');
    sep = max(sep, abs(d*a') - ra - rb);
end
end
