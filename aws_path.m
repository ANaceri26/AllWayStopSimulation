function pth = aws_path(P, approach, turn)
%AWS_PATH  Centre-line path of one movement through the intersection.
%   approach : 0 = from South (driving north), 1 = from East, 2 = from North,
%              3 = from West
%   turn     : 'S' straight, 'L' left, 'R' right
%   pth.pts  : Kx2 points,  pth.s : arc length,  pth.sLine : s at the stop
%   line,  pth.sExit : s where the path leaves the intersection box.

w = P.w;  b = P.b;  L = P.L;  ds = P.ds;

% --- built for a car coming from the South, then rotated -------------------
n1   = round((L - b)/ds);
y    = linspace(-L, -b, n1 + 1)';
seg1 = [repmat(w/2, n1 + 1, 1), y];

switch upper(turn)
    case 'S'
        nb   = round(2*b/ds);
        yy   = linspace(-b, b, nb + 1)';
        seg2 = [repmat(w/2, nb, 1), yy(2:end)];
        d    = [0 1];
        Rad  = inf;
    case 'R'
        Rad  = b - w/2;
        nb   = ceil(Rad*pi/2/ds);
        th   = linspace(pi, pi/2, nb + 1)';
        th   = th(2:end);
        seg2 = [b + Rad*cos(th), -b + Rad*sin(th)];
        d    = [1 0];
    case 'L'
        Rad  = b + w/2;
        nb   = ceil(Rad*pi/2/ds);
        th   = linspace(0, pi/2, nb + 1)';
        th   = th(2:end);
        seg2 = [-b + Rad*cos(th), -b + Rad*sin(th)];
        d    = [-1 0];
    otherwise
        error('aws_path:turn', 'turn must be ''S'', ''L'' or ''R''.');
end

n3   = round(P.Lout/ds);
seg3 = repmat(seg2(end, :), n3, 1) + (1:n3)'*ds*d;
pts  = [seg1; seg2; seg3];

ang = approach*pi/2;
c   = cos(ang);  s = sin(ang);
pts = pts*[c s; -s c];

sc = [0; cumsum(hypot(diff(pts(:, 1)), diff(pts(:, 2))))];

pth.pts   = pts;
pth.s     = sc;
pth.iLine = n1 + 1;
pth.iExit = n1 + nb + 1;
pth.sLine = sc(pth.iLine);
pth.sExit = sc(pth.iExit);
pth.R     = Rad;
end
