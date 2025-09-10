function set_fmu_parameters_for_run(fmuParams, output_filepath)
    % Function to set the parameters for the FMUs and print the set values

    if exist(output_filepath, 'file')
        error(['The file ', output_filepath, ' already exists. Delete the file or create a new foler name']);
    end

    [outputDir, ~, ~] = fileparts(output_filepath);
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);  % Create the directory
        disp(['Directory created: ', outputDir]);
    end

    parameterData = {};  % used for storing the parameters before writing them to file
    
    % Loop through the FMUs and set their parameters
    fmuNames = fieldnames(fmuParams); % Get list of FMU names

    % Open file for writing formatted output
    fileID = fopen(output_filepath, 'w');
    
    for i = 1:length(fmuNames)
        fmuName = fmuNames{i};  % Get current FMU name
        
        % Map the valid field name back to the FMU name in Simulink
        switch fmuName
            case 'WindModel'
                displayName = 'Wind model';
            case 'WaveForces'
                displayName = 'Wave forces';
            case 'Zeabuz'
                displayName = 'Zeabuz 1';
            case 'DistanceSensorZT1FSS'
                displayName = 'Distance Sensor ZT1 FS-S';
            case 'DistanceSensorZT2FSF'
                displayName = 'Distance Sensor ZT2 FS-F';
            case 'DistanceSensorZT3FPF'
                displayName = 'Distance Sensor ZT3 FP-F';
            case 'DistanceSensorZT4FPP'
                displayName = 'Distance Sensor ZT4 FP-P';
            case 'DistanceSensorZT5APP'
                displayName = 'Distance Sensor ZT5 AP-P';
            case 'DistanceSensorZT6APA'
                displayName = 'Distance Sensor ZT6 AP-A';
            case 'DistanceSensorZT7ASA'
                displayName = 'Distance Sensor ZT7 AS-A';
            case 'DistanceSensorZT8ASS'
                displayName = 'Distance Sensor ZT8 AS-S';
            otherwise
                displayName = fmuName;  % Default case, no change
        end
        
        paramValues = fmuParams.(fmuName);  % Get its parameters
        paramNames = fieldnames(paramValues);  % Extract parameter names

        % Write FMU name before listing its parameters
        fprintf(fileID, '\n%s\n', fmuName);  % Blank line before new FMU name
        fprintf(fileID, '--------------------\n');  % Separator for clarity
    
        % Loop through each parameter and set it in the FMU block
        for j = 1:length(paramNames)
            paramName = paramNames{j};  % Parameter name
            paramValue = paramValues.(paramName);  % Parameter value
    
            % Set the parameter in the Simulink FMU block
            blockPath = ['fmu_simulation/', displayName];  % Full path to the FMU block
            try
                % Set the parameter for the FMU block
                % if strcmp(fmuName, 'Zeabuz') && startsWith(paramName, 'quay_')
                if startsWith(paramName, 'quay_')
                    % If parameter is 'quay_i_en', set directly without mat2str
                    set_param(blockPath, paramName, paramValue);
                else
                    % Use mat2str for all other cases
                    set_param(blockPath, paramName, mat2str(paramValue));
                end
    
                % Display confirmation message
                % disp(['Set ', paramName, ' to ', mat2str(paramValue), ' in FMU: ', displayName]);
            catch ME
                % If an error occurs, display an error message
                disp(['Error setting parameter ', paramName, ' in FMU: ', displayName]);
                disp(['Error: ', ME.message]);
            end

            % Convert parameter value to string for formatted output
            if isnumeric(paramValue)  % If numeric, convert array to string
                paramValueStr = num2str(paramValue);
            else
                paramValueStr = char(paramValue);
            end

            % Print formatted output with indentation
            fprintf(fileID, '    %s: %s\n', paramName, paramValueStr);

            % Store for debugging or optional CSV output
            parameterData = [parameterData; {fmuName, paramName, paramValueStr}];
        end

        % Add a blank line between FMUs for readability
        fprintf(fileID, '\n');
    end

    % Close file
    fclose(fileID);
    disp(['Formatted FMU parameters saved to: ', output_filepath]);
end

