% Define your full file path (customize to your system)
filepath = fullfile('../../masteroppgave-rapport/results', controller_type, batch, strcat('run', num2str(run)), run_description, 'wind_forces.csv');

time = out.wind_forces.time;
data = out.wind_forces.signals.values;
data_3dof = data(:, [1, 2, 6]);

output = [time, data_3dof];

% Open file and write header manually
fid = fopen(filepath, 'w');
fprintf(fid, 't,X,Y,N\n');  % <-- Your header row
fclose(fid);

% Append data below the header
dlmwrite(filepath, output, '-append');