%% Activate the Python environment
if exist('python_env', 'var')
    python_env.terminate;
    clear python_env;
end
python_env_path = 'C:\Users\rkhan\docking\Scripts\python.exe'; % Update this path to your Python environment
python_env = pyenv('Version', python_env_path, 'ExecutionMode', 'OutOfProcess');




%% Define Simulation Settings
controller_type = "pid"; % Options: "pid", "mpc"
batch = "no_disturbances"; % Options: "no_disturbances", "small_disturbances", "moderate_disturbances", "large_disturbances"
run = 1;
run_description = 'speed11_dir264_left';
quay_type = "l_quay";  % Options: "l_quay", "h_quay"

mean_wind_dir = 0;  % 1.48004619 (push boat toward quay), 4.62163884 (push boat away from quay)
wind_dir_noise_pwr = 0;
wind_dir_time_const = 10;
mean_wind_speed = 0;
wind_speed_noise_pwr = 0;
wind_speed_time_const = 10;

wave_force_stddev = [0, 0, 0, 0, 0, 0];

init_pose = [100, 484, 0, 0, 0, 2.4]; % [130, 470, 0, 0, 0, 2.4] left-hand-side. [130, 505, 0, 0, 0, 3.9] right-hand-side (possibly 510)
init_velocity = [0.5, 0, 0, 0, 0, 0];




if ~isempty(run_description)
    output_filepath = fullfile('../../masteroppgave-rapport/results', controller_type, batch, strcat('run', num2str(run)), run_description, 'fmu_params.csv');
else
    output_filepath = fullfile('../../masteroppgave-rapport/results', controller_type, batch, strcat('run', num2str(run)), 'fmu_params.csv');
end

% Set quay enable flag
if quay_type == "l_quay"
    enable_l = 1;
else
    enable_l = 0;
end

%% Define FMU Parameters
fmuParams = struct(...
    'WindModel', struct('mean_wind_direction', mean_wind_dir, 'wind_direction_noise_power', wind_dir_noise_pwr, 'wind_direction_time_constant', wind_dir_time_const,...
                        'mean_wind_speed', mean_wind_speed, 'wind_speed_noise_power', wind_speed_noise_pwr, 'wind_speed_time_constant', wind_speed_time_const),...
    'WaveForces', struct('wave_force_stddev', wave_force_stddev), ...
    'Zeabuz', struct('initial_position', init_pose, 'initial_velocity', init_velocity, 'quay_1_en', 1, 'quay_2_en', enable_l)); %,...
    % 'DistanceSensorZT1FSS', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT2FSF', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT3FPF', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT4FPP', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT5APP', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT6APA', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT7ASA', struct('quay_1_en', 1, 'quay_2_en', enable_l),...
    % 'DistanceSensorZT8ASS', struct('quay_1_en', 1, 'quay_2_en', enable_l)...
% );

%% Assign Variables to MATLAB Workspace (For MATLAB Function Block Access)
assignin('base', 'batch', batch);
assignin('base', 'run', run);
assignin('base', 'quay_type', quay_type);
assignin('base', 'controller_type', controller_type);
assignin('base', 'output_filepath', output_filepath);
assignin('base', 'fmuParams', fmuParams);

%% Load the Simulink Model and Set FMU Parameters
model_name = 'fmu_simulation'; % Update with your actual Simulink model name
load_system(model_name);

% Call function to set FMU parameters
set_fmu_parameters_for_run(fmuParams, output_filepath);

% Save the model (optional)
save_system(model_name);

%% Start the Simulation
% sim(model_name);




%%

%docking_python = py.importlib.import_module('docking_python');
%extended_dp_code = py.importlib.import_module('docking_python.docking_algorithms.extended_dp_controller');
%dp_controller = extended_dp_code.ExtendedDpDocking();

%current_pyenv = pyenv;

% Check if the environment is activated correctly
%if ~strcmp(current_pyenv.Version, python_env_path)
%    pyenv('Version', python_env_path); 
%end


%% Setting paths
% Define the root directory for your project
%projectRoot = 'C:\Users\rkhan\Autonomous-docking';

% Add the FMU directory to the MATLAB path
%addpath(fullfile(projectRoot, 'fmus'));

% Add the MATLAB related folder if you have MATLAB files there
%addpath(fullfile(projectRoot, 'matlab'));

% If your Python code is interfacing with the FMU or Simulink,
% add the Python code folder to MATLAB path
%addpath(fullfile(projectRoot, 'docking_python'));

% Add the 'fmu_wrappers' folder (which contains helper functions)
%addpath(fullfile(projectRoot, 'docking_python', 'fmu_wrappers'));

% Optionally, add the 'docking_algorithms' folder
%addpath(fullfile(projectRoot, 'docking_python', 'docking_algorithms'));

% Save the path for future MATLAB sessions (optional)
%savepath;


%% Delete the specified folder if it exists
% Only necessary when running docking code as FMU
% folderToDelete = fullfile(projectRoot, 'slprj\_fmu', '281c5070579b1a39a9384938d89be35c');  % Specify the folder path
% 
% if exist(folderToDelete, 'dir')  % Check if the folder exists
%     rmdir(folderToDelete, 's');  % Remove the folder and its contents
%     fprintf('Deleted folder: %s\n', folderToDelete);  % Optional: Print confirmation message
% else
%     fprintf('Folder does not exist: %s\n', folderToDelete);  % Optional: Print warning message
% end

%%
%python_code = py.importlib.import_module('docking_algorithms.extended_dp_docking');
%sum_code = py.importlib.import_module('docking_algorithms.sum');

