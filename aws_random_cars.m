function cars = aws_random_cars(rate, T, pLeft, pRight)
%AWS_RANDOM_CARS  Random (Poisson) arrivals on all four approaches.
%   cars = aws_random_cars(rate, T, pLeft, pRight)

if nargin < 3, pLeft  = 0.2; end
if nargin < 4, pRight = 0.2; end

cars = struct('approach', {}, 'turn', {}, 'tSpawn', {}, 'dStart', {}, 'type', {});
for a = 0:3
    t = 0;
    while true
        t = t - log(rand)/rate;
        if t > T, break, end
        u = rand;
        if u < pLeft
            turn = 'L';
        elseif u < pLeft + pRight
            turn = 'R';
        else
            turn = 'S';
        end
        n = numel(cars) + 1;
        cars(n).approach = a;
        cars(n).turn     = turn;
        cars(n).tSpawn   = t;
        cars(n).dStart   = 40;
        cars(n).type     = 'av';
    end
end
if ~isempty(cars)
    [~, idx] = sort([cars.tSpawn]);
    cars = cars(idx);
end
end
