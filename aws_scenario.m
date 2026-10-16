function [cars, peds, P, expect, info] = aws_scenario(name)
%AWS_SCENARIO  Library of all-way-stop test scenarios.
%   [cars, peds, P, expect, info] = aws_scenario(name)
%   names = aws_scenario('list')      returns all scenario names
%
%   expect  what a correct run must look like (used by aws_check)
%   info    one-line description

names = {'single', 'fifo', 'tie_right', 'head_on_left', 'concurrent', ...
         'deadlock4', 'queue', 'pedestrian', 'runner', 'false_start', 'rush_hour'};
if nargin < 1 || strcmpi(name, 'list')
    cars = names;
    return
end

P      = aws_params();
peds   = struct('p0', {}, 'p1', {}, 't0', {}, 'speed', {});
expect = struct();

switch lower(name)
    case 'single'
        info = 'One car: must come to a full stop, wait, then go.';
        cars = mk('S', 'S', 0);
        expect.order = 1;

    case 'fifo'
        info = 'Four cars arrive one after another: first to stop goes first.';
        cars = [mk('S', 'S', 0), mk('E', 'S', 1.5), mk('N', 'S', 3), mk('W', 'S', 4.5)];
        expect.order = [1 2 3 4];

    case 'tie_right'
        info = 'Two cars stop at the same instant: the one on the right goes first.';
        cars = [mk('S', 'S', 0), mk('E', 'S', 0)];
        expect.order = [2 1];

    case 'head_on_left'
        info = 'Head-on tie: the car going straight goes before the left-turner.';
        cars = [mk('S', 'L', 0), mk('N', 'S', 0)];
        expect.order = [2 1];

    case 'concurrent'
        info = 'Non-conflicting movements share the box (opposite straights, then two right turns).';
        cars = [mk('S', 'S', 0), mk('N', 'S', 0), mk('E', 'R', 2), mk('W', 'R', 2)];
        expect.order = [1 2 3 4];
        expect.together = [1 2; 3 4];

    case 'deadlock4'
        info = 'Four cars stop simultaneously: everyone yields to the right -> deadlock, broken by a token.';
        cars = [mk('S', 'S', 0), mk('E', 'S', 0), mk('N', 'S', 0), mk('W', 'S', 0)];
        expect.order = [1 4 3 2];

    case 'queue'
        info = 'Three cars per approach with mixed turns: approaches take turns fairly.';
        turns = 'SLRSSLSRSSLS';
        dirs  = 'SENW';
        cars  = mk('S', 'S', 0);
        k = 0;
        for r = 0:2
            for a = 1:4
                k = k + 1;
                cars(k) = mk(dirs(a), turns(k), 2.0*r + 0.4*(a - 1));
            end
        end
        expect.maxDelay = 45;

    case 'pedestrian'
        info = 'A pedestrian steps onto the crosswalk while a car is already turning right.';
        cars = [mk('W', 'R', 0), mk('S', 'S', 2.5), mk('N', 'L', 3)];
        peds(1).p0 = [-5.5 -4.7];  peds(1).p1 = [9 -4.7];
        peds(1).t0 = 7.5;          peds(1).speed = 1.4;
        expect.order  = [1 2 3];
        expect.minSep = 0.5;

    case 'runner'
        info = 'A human driver (red) runs the stop sign; the AVs detect it and hold.';
        cars = [mk('S', 'S', 0), mk('N', 'L', 0.5), mk('E', 'S', 3.6, 'runner'), mk('W', 'S', 3.0)];
        expect.order = [3 1 2 4];

    case 'false_start'
        info = 'No V2X + a wrong sensor estimate: both cars think it is their turn; one backs off.';
        cars = [mk('S', 'S', 0), mk('E', 'S', 0)];
        P.v2x = false;
        P.noiseOverride = [0 0.5; 0 0];   % car 1 thinks car 2 stopped 0.5 s later
        expect.order      = [1 2];
        expect.falseStart = 1;

    case 'rush_hour'
        info = 'Random traffic from all four sides for 40 s.';
        rng(7);
        cars = aws_random_cars(0.08, 40, 0.2, 0.2);
        P.Tend = 240;

    otherwise
        error('aws_scenario:name', 'Unknown scenario "%s". Use aws_scenario(''list'').', name);
end
end

% ----------------------------------------------------------------------------
function c = mk(dirChar, turn, tSpawn, type)
if nargin < 4, type = 'av'; end
c.approach = find('SENW' == dirChar) - 1;
c.turn     = turn;
c.tSpawn   = tSpawn;
c.dStart   = 40;
c.type     = type;
end
