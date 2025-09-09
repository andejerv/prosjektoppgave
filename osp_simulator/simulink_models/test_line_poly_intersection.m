% Tests from af_utils/geometry/test/test_line_poly_intersection.py
%% test_intersection()
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [0, 10]
]';
p0 = [-5, 0]';
p1 = [15, 5]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[0, 5/4]', [10, (5*3)/4]'], 'all'))
assert(all(t == [0.25, 0.75], 'all'))

%% test_parallel_no_intersection():
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [0, 10]
]';

p0 = [11, -5]';
p1 = [11, 15]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [], 'all'))
assert(all(t == [], 'all'))

%% test_parallel_intersection():
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [0, 10]
]';

% Line touching a point
p0 = [10, -5]';
p1 = [10, 0]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [p1, p1], 'all'))
assert(all(t == [1, 1], 'all'))

% Line passing entire segment
p0 = [10, -5]';
p1 = [10, 15]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[10, 0]', [10, 10]'], 'all'))
assert(all(t == [0.25, 0.75], 'all'))

% Line ending inside segment
p0 = [10, -5]';
p1 = [10, 5]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[10, 0]', [10, 5]'], 'all'))
assert(all(t == [0.5, 1], 'all'))

% Line ending inside polygon
p0 = [5, -5]';
p1 = [5, 5]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[5, 0]', [5, 5]'], 'all'))
assert(all(t == [0.5, 1], 'all'))

%% test_inside_polygon():
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [0, 10]
]';

p0 = [2.5, 7.5]';
p1 = [9, 1]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[2.5, 7.5]', [9, 1]'], 'all'))
assert(all(t == [0, 1], 'all'))

%% test_zero_area_polygon():
% NOT IMPLEMENTED DUE TO LINE POLYGON SUPPORT NOT IMPLEMENTED

%% test_almost_parallel():
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [0, 10]
]';

thr = 1e-9;

% Intersects, starts on "correct side"
p0 = [0, 10-thr/100]';
p1 = [10, 10+thr/100]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(abs(intersection(1,1)- 0) < thr)
assert(abs(intersection(1,2) - 10) < thr)
assert(all(t == [0, 1], 'all'))

% Intersects, starts on "wrong side"
p0 = [0, 10+thr/100]';
p1 = [10, 10-thr/100]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == []))
assert(all(t == []))

% Slightly inside
p0 = [5, 10-thr/100]';
p1 = [10, 10-2*thr/100]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(intersection(1,1) == 5)
assert(intersection(1,2) == 10)
assert(all(t == [0, 1], 'all'))

%% test_single_point():
v = [
    [0, 0]
    [10, 0]
    [10, 10]
    [5, 10]
]';

p0 = [0, -5]';
p1 = [0, 5]';

[intersection, t] = line_poly_intersection(p0, p1, v);

assert(all(intersection == [[0, 0]', [0,0]'],'all'))
assert(all(t == [0.5, 0.5], 'all'))
