function P = aws_params()
%AWS_PARAMS  Default parameters for the all-way-stop autonomous-car simulator.
%   All units SI (m, s, m/s).  Change anything here or override after the call:
%       P = aws_params();  P.v2x = false;  P.sensorSigma = 0.8;

% --- geometry (right-hand traffic, one lane per direction) -----------------
P.w        = 3.5;    % lane width
P.b        = 6.0;    % stop line distance from the intersection centre
P.L        = 50.0;   % length of each approach road
P.Lout     = 35.0;   % length of each exit road
P.ds       = 0.25;   % path sampling step

% --- vehicle ---------------------------------------------------------------
P.carLen   = 4.5;
P.carWid   = 1.8;
P.vCruise  = 10.0;   % 36 km/h
P.aMax     = 2.0;    % max acceleration
P.bComf    = 2.5;    % comfortable braking
P.bMax     = 7.0;    % emergency braking
P.aLat     = 2.0;    % lateral acceleration limit while turning
P.tau      = 0.3;    % speed-tracking time constant
P.s0       = 2.0;    % standstill gap to the car in front
P.stopMargin = 0.5;  % stop this far before the stop line

% --- stop-sign behaviour ---------------------------------------------------
P.dwell      = 1.0;  % mandatory full-stop time before a car may go
P.tieTol     = 0.3;  % arrivals closer than this count as "simultaneous"
P.abortDist  = 1.2;  % a false start can still be aborted this far past the line
P.conflictDist = 2.6;% two movements conflict if their paths get closer than this
P.obsDist    = 1.8;  % obstacle is "on my path" if closer than this to it
P.lookAhead  = 25.0; % obstacle look-ahead distance along the path

% --- perception / communication --------------------------------------------
P.v2x         = true;  % true : cars share exact stop times over V2X
                       % false: each car estimates the others with its sensors
P.sensorSigma = 0.0;   % std-dev (s) of the arrival-time estimate when v2x=false
P.noiseOverride = [];  % optional NxN matrix, (i,j) = error of car i about car j

% --- simulation ------------------------------------------------------------
P.dt    = 0.05;
P.Tend  = 120.0;
P.seed  = 1;
end
