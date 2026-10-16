%DEMO_EMOTOR_FOC  Field-oriented control of a traction motor vs. its requirement.
%   Idea from the MathWorks talk on e-motor software control: the digital
%   thread starts at a requirement and ends at a test that checks it.
%
%   Requirement (from the talk): "The achieved torque should reach commanded
%   torque within 10 ms with overshoot not exceeding 5 % of commanded torque."
%
%   Plant : PMSM in the rotor (d-q) frame, integrated at 10 us
%   Control: PI current controllers with decoupling and anti-windup,
%            sampled at 100 us (the rate mentioned in the talk)
%   Test  : 0 -> 100 Nm torque step at 1500 rpm (pulling away from the stop
%           sign), then the requirement is checked automatically.
%   Base MATLAB only.

% --- motor (typical values for a small traction machine) --------------------
Rs = 0.012;  Ld = 0.15e-3;  Lq = 0.35e-3;  lam = 0.06;  p = 4;
Vdc = 350;   Vmax = Vdc/sqrt(3);
rpm = 1500;  we = rpm*2*pi/60*p;               % electrical speed (rad/s)

% --- controller --------------------------------------------------------------
Ts  = 1e-4;  wbw = 2000;                       % sample time, bandwidth (rad/s)
Kpd = wbw*Ld;  Kpq = wbw*Lq;  Ki = wbw*Rs;

% --- simulation ---------------------------------------------------------------
dt = 1e-5;  Tend = 0.03;  tStep = 0.005;  Tref = 100;
n  = round(Tend/dt);  nCtl = round(Ts/dt);
t  = (0:n - 1)'*dt;
Tcmd = Tref*(t >= tStep);
id = 0;  iq = 0;  xd = 0;  xq = 0;  vd = 0;  vq = 0;
Tq = zeros(n, 1);  ID = zeros(n, 1);  IQ = zeros(n, 1);  VM = zeros(n, 1);
for k = 1:n
    if mod(k - 1, nCtl) == 0                   % ---- controller task (100 us)
        iqRef = Tcmd(k)/(1.5*p*lam);  idRef = 0;
        eD = idRef - id;  eQ = iqRef - iq;
        ud = Kpd*eD + xd - we*Lq*iq;                   % PI + decoupling
        uq = Kpq*eQ + xq + we*(Ld*id + lam);
        mag = hypot(ud, uq);
        if mag > Vmax                                  % inverter voltage limit
            ud = ud*Vmax/mag;  uq = uq*Vmax/mag;       % (integrators frozen)
        else
            xd = xd + Ki*Ts*eD;  xq = xq + Ki*Ts*eQ;
        end
        vd = ud;  vq = uq;
    end
    did = (vd - Rs*id + we*Lq*iq)/Ld;          % ---- motor (10 us)
    diq = (vq - Rs*iq - we*(Ld*id + lam))/Lq;
    id  = id + did*dt;
    iq  = iq + diq*dt;
    Tq(k) = 1.5*p*(lam*iq + (Ld - Lq)*id*iq);
    ID(k) = id;  IQ(k) = iq;  VM(k) = hypot(vd, vq);
end

% --- check the requirement ---------------------------------------------------
after     = t >= tStep;
overshoot = max(0, (max(Tq) - Tref)/Tref*100);
outside   = find(after & abs(Tq - Tref) > 0.02*Tref, 1, 'last');
if isempty(outside), tSettle = 0; else, tSettle = (t(outside) + dt - tStep)*1000; end
passed = (tSettle <= 10) && (overshoot <= 5);
fprintf('\nTorque step 0 -> %d Nm at %d rpm\n', Tref, rpm);
fprintf('  settling time (2 %% band): %5.2f ms   (requirement: <= 10 ms)\n', tSettle);
fprintf('  overshoot               : %5.2f %%    (requirement: <= 5 %%)\n', overshoot);
if passed, fprintf('  Requirement: PASS\n\n'); else, fprintf('  Requirement: FAIL\n\n'); end

figure('Color', 'w', 'Name', 'E-motor FOC', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(t*1000, Tcmd, 'k--', t*1000, Tq, 'b', 'LineWidth', 1.3);  hold on;
plot([tStep tStep + 0.010]*1000, [1.05 1.05]*Tref, 'r:', 'LineWidth', 1.2);
grid on;  ylabel('torque (Nm)');  legend({'command', 'achieved', '+5 % limit, 10 ms window'}, 'Location', 'southeast');
title(sprintf('Torque response: settles in %.2f ms, overshoot %.2f %%', tSettle, overshoot));
subplot(3, 1, 2);
plot(t*1000, ID, t*1000, IQ, 'LineWidth', 1.3);
grid on;  ylabel('current (A)');  legend({'i_d', 'i_q'}, 'Location', 'southeast');
subplot(3, 1, 3);
plot(t*1000, VM, 'LineWidth', 1.3);  hold on;
plot([0 Tend]*1000, [Vmax Vmax], 'r--');
grid on;  ylabel('voltage (V)');  xlabel('time (ms)');  legend({'|v_{dq}|', 'inverter limit'}, 'Location', 'southeast');
