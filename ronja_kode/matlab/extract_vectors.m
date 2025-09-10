function [x, y, psi] = extract_vectors(cell_array)
    n = length(cell_array);
    x = zeros(n, 1);
    y = zeros(n, 1);
    psi = zeros(n, 1);

    for i = 1:n
        values = strsplit(cell_array{i}, ',');
        x(i) = str2double(values{1});
        y(i) = str2double(values{2});
        psi(i) = str2double(values{3});
    end
end