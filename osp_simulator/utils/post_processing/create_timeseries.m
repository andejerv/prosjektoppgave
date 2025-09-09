function timeseries = create_timeseries(path)
    data = readmatrix(path);
    fid = fopen(path);
    header = fgets(fid);
    fclose(fid);
    label_split = split(header, ',');
    timeseries = containers.Map;
    for i = 1:length(label_split)
        splitted = split(label_split(i));
        label = splitted(1);
        timeseries(label{1}) = data(:,i);
    end
end

