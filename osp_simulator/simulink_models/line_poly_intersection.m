function [intersection, t] = line_poly_intersection(p0, p1, v)
% line_poly_intersection
%    Find intersection between a line segment P(t) = p0 + t(p1-p0) for
%    t in [0,1] and a convex polygon.
%
%    Based on http://geomalgorithms.com/a13-_intersect-4.html
%
%    :param p0: Line segment start point dimension (2, 1)
%    :param p1: Line segment end point dimension (2, 1)
%    :param v: Polygon vertices (dimension (2, N) for a N-sided polygon), with
%        vertices sorted in counterclockwise order.
%    :return: Intersection points and corresponding t parameter.

thr=1e-9;

p10 = p1 - p0;
t_e = 0;
t_l = 1;

v_size = size(v);

% default output
intersection = [];
t = [];

% Loop over polygon vertices
for i = 0:(v_size(2) - 1)
    i0 = mod(i-1, v_size(2))+1;
    i1 = mod(i, v_size(2))+1;
    ev = v(:, i1) - v(:, i0);
    n = [ev(2), -ev(1)];

    N = - dot((p0 - v(:, i0)), n);
    D = dot(p10, n);

    if abs(D) < thr
        % Parallel line
        if N < 0
            % Line from Vi to p0 points outward from edge - line does not
            % intersect with polygon
            return
        end
        continue
    else
        % Non-parallel line
        t_ = N / D;

        if D < 0
            % p0->p1 enters through edge Vi->Vi+1
            t_e = max(t_e, t_);
            if t_e > t_l
                % Line segment enters after exiting polygon
                return
            end
        else
            % p0->p1 leaves through edge Vi->Vi+1
            t_l = min(t_l, t_);
            if t_l < t_e
                % Line segment leaves before entering polygon
                return
            end
        end

    end
end
    
% Here, we have t_e <= t_l and a valid intersection
I1 = p0 + t_e * p10;
I2 = p0 + t_l * p10;

intersection = [I1, I2];
t = [t_e, t_l];
end