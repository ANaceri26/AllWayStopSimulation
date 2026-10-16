classdef AllWayStopTest < matlab.unittest.TestCase
    %ALLWAYSTOPTEST  Unit tests for the all-way-stop simulator.
    %   results = runtests('AllWayStopTest');
    %   table(results)

    properties (TestParameter)
        scenario = {'single', 'fifo', 'tie_right', 'head_on_left', 'concurrent', ...
                    'deadlock4', 'queue', 'pedestrian', 'runner', 'false_start', 'rush_hour'};
        seed     = {1, 2, 3, 4};
        movement = {'S', 'L', 'R'};
    end

    methods (Test)
        function scenarioBehavesAsDesigned(testCase, scenario)
            [cars, peds, P, expect] = aws_scenario(scenario);
            R   = aws_simulate(P, cars, peds);
            res = aws_check(R, expect);
            testCase.verifyTrue(res.pass, res.text);
        end

        function scenarioIsSafeWithNoisySensors(testCase, scenario)
            [cars, peds, P] = aws_scenario(scenario);
            P.v2x = false;  P.sensorSigma = 0.6;  P.noiseOverride = [];
            R   = aws_simulate(P, cars, peds);
            res = aws_check(R, struct());
            testCase.verifyTrue(res.pass, res.text);
        end

        function randomTrafficIsSafeAndLive(testCase, seed)
            P = aws_params();
            P.seed = seed;  P.Tend = 240;  P.v2x = false;  P.sensorSigma = 1.0;
            rng(200 + seed);
            cars = aws_random_cars(0.08, 30, 0.25, 0.25);
            testCase.assumeNotEmpty(cars);
            R   = aws_simulate(P, cars, []);
            res = aws_check(R, struct());
            testCase.verifyTrue(res.pass, res.text);
        end

        function everyCarMakesAFullStop(testCase, movement)
            P = aws_params();
            car = struct('approach', 0, 'turn', movement, 'tSpawn', 0, 'dStart', 40, 'type', 'av');
            R = aws_simulate(P, car, []);
            testCase.verifyFalse(isnan(R.tArr), 'car never stopped');
            testCase.verifyGreaterThanOrEqual(R.tGo - R.tArr, P.dwell - 1e-9, 'dwell too short');
            testCase.verifyEqual(R.nStopViolations, 0);
        end

        function pathsAreContinuous(testCase, movement)
            P = aws_params();
            for a = 0:3
                pth  = aws_path(P, a, movement);
                step = hypot(diff(pth.pts(:, 1)), diff(pth.pts(:, 2)));
                testCase.verifyLessThan(max(step), P.ds + 1e-6);
                testCase.verifyGreaterThan(min(step), 0);
                testCase.verifyEqual(pth.sLine, P.L - P.b, 'AbsTol', 1e-9);
            end
        end
    end
end
