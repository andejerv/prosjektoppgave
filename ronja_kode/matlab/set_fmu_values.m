% Define list of FMUs that are part of xml but not simulink diagram
excludedFmuNames = {'mooring', 'advanced_navigation_gps_compass', 'dp_interface', 'advanced_navigation', 'hatch_control', 'zb_sim_clock', 'DP Mock'};
    % Skipping DP Mock due to zero-initialization and variable names with
    % dots in it -> enter values manually

% Step 1: Load and parse the XML file
xmlFile = 'fmu-config.xml';  % Path to your XML file
xmlDoc = xmlread(xmlFile);


% Step 2: Get all FMU elements
fmuList = xmlDoc.getElementsByTagName('Simulator');


% Step 3: Loop through each FMU element
for i = 0:fmuList.getLength-1
    fmuElement = fmuList.item(i);
    
    % Get the FMU name and source attribute (optional, but may be useful)
    fmuName = char(fmuElement.getAttribute('name'));
    fmuSource = char(fmuElement.getAttribute('source'));

    % Check if the FMU is in the excluded list
    if ismember(fmuName, excludedFmuNames)
        disp(['Skipping FMU: ', fmuName]);
        continue;  % Skip this FMU and go to the next one
    end
    
    % Get the 'InitialValues' element
    valuesElement = fmuElement.getElementsByTagName('InitialValues').item(0);
    
    % Get all 'InitialValue' elements (parameters)
    valueList = valuesElement.getElementsByTagName('InitialValue');

    % Initialize empty structs to store vector and matrix parameters
    vectorParams = struct();
    matrixParams = struct();

    
    % Step 4: Loop through each 'InitialValue' element and get the variable name and value
    for j = 0:valueList.getLength-1
        valueElement = valueList.item(j);
        
        % Get the variable name
        variableName = char(valueElement.getAttribute('variable'));
        % Following variables do not exist -> skip
        if strcmp(fmuName, "DP Mock") && (strcmp(variableName, "enableReadyForAutonomy") || strcmp(variableName, "desiredControlMode"))
            continue;
        end
        
        % Check if the variable is a vector component
        vectorMatch = regexp(variableName, '^([a-zA-Z_]\w*)\[(\d+)\]$', 'tokens');
        
        % Check if the variable is a matrix component
        matrixMatch = regexp(variableName, '^([a-zA-Z_]\w*)\[(\d+),(\d+)\]$', 'tokens');
        

        if ~isempty(vectorMatch)
            % It's part of a vector
            vectorBase = vectorMatch{1}{1};  % Base name 
            index = str2double(vectorMatch{1}{2});  % Index (e.g., 1, 2, 3)
            
            % Check for Real values (assuming all vector elements are Real for simplicity)
            realElement = valueElement.getElementsByTagName('Real');
            if realElement.getLength > 0
                % Get the value and store it in the vector
                realValue = str2double(realElement.item(0).getAttribute('value'));
                
                % Store the value in the vectorParams struct under the vectorBase name
                if ~isfield(vectorParams, vectorBase)
                    vectorParams.(vectorBase) = [];
                end
                vectorParams.(vectorBase)(index) = realValue;
            end
        
        elseif ~isempty(matrixMatch)
            % It's part of a matrix (e.g., "param[1,1]", "param[1,2]")
            matrixBase = matrixMatch{1}{1};  % Base name (e.g., "param")
            rowIndex = str2double(matrixMatch{1}{2});  % Row index (e.g., 1)
            colIndex = str2double(matrixMatch{1}{3});  % Column index (e.g., 1, 2, 3)
            
            % Check for Real values (assuming all matrix elements are Real for simplicity)
            realElement = valueElement.getElementsByTagName('Real');
            if realElement.getLength > 0
                % Get the value and store it in the matrix
                realValue = str2double(realElement.item(0).getAttribute('value'));
                
                % Store the value in the matrixParams struct under the matrixBase name
                if ~isfield(matrixParams, matrixBase)
                    matrixParams.(matrixBase) = [];  % Initialize empty matrix
                end
                matrixParams.(matrixBase)(rowIndex, colIndex) = realValue;  % Store value at specific row, col
            end
         
        else
            % Handle scalar parameters
            % Check if it's a Real, Boolean, or Integer
            realElement = valueElement.getElementsByTagName('Real');
            booleanElement = valueElement.getElementsByTagName('Boolean');
            integerElement = valueElement.getElementsByTagName('Integer');
            
            if realElement.getLength > 0
                % Scalar Real value
                realValue = str2double(realElement.item(0).getAttribute('value'));
                set_param(['fmu_simulation/', fmuName], variableName, num2str(realValue));
                disp(['Set scalar Real parameter ', variableName, ' to ', num2str(realValue), ' for FMU ', fmuName]);

            elseif booleanElement.getLength > 0
                % Scalar Boolean value
                boolValue = booleanElement.item(0).getAttribute('value');
                % Translate 'true'/'false' from XML to 'on'/'off' for Simulink checkbox
                if strcmpi(boolValue, 'true')
                    checkboxValue = 'on';  % Simulink expects 'on' for checked
                else
                    checkboxValue = 'off';  % Simulink expects 'off' for unchecked
                end
                
                % Set the checkbox parameter in the FMU block
                try
                    set_param(['fmu_simulation/', fmuName], variableName, checkboxValue);
                    disp(['Set checkbox parameter ', variableName, ' to ', checkboxValue, ' for FMU ', fmuName]);
                catch ME
                    % Error handling
                    disp(['Failed to set checkbox parameter ', variableName, ' for FMU ', fmuName]);
                    disp(['Error: ', ME.message]);
                end


            elseif integerElement.getLength > 0
                % Scalar Integer value
                intValue = integerElement.item(0).getAttribute('value');
                set_param(['fmu_simulation/', fmuName], variableName, intValue);
                disp(['Set scalar Integer parameter ', variableName, ' to ', intValue, ' for FMU ', fmuName]);
            end
        end
    end
    
    % Step 5: Set vector parameters in Simulink FMU blocks
    paramNames = fieldnames(vectorParams);
    for k = 1:length(paramNames)
        vectorName = paramNames{k};  % e.g., "velocity" or "acceleration"
        vectorValue = vectorParams.(vectorName);  % Extract the full vector
        
        % Set the vector as a whole in Simulink (e.g., "velocity" or "acceleration")
        set_param(['fmu_simulation/', fmuName], vectorName, mat2str(vectorValue));
        disp(['Set vector parameter ', vectorName, ' to ', mat2str(vectorValue), ' for FMU ', fmuName]);
    end
    
    % Set matrix parameters in Smulink FMU blocks
    matrixNames = fieldnames(matrixParams);
    for k = 1:length(matrixNames)
        matrixName = matrixNames{k};  % e.g., "param"
        matrixValue = matrixParams.(matrixName);  % Extract the full matrix
        
        % Set the matrix as a whole in Simulink
        set_param(['fmu_simulation/', fmuName], matrixName, mat2str(matrixValue));
        disp(['Set matrix parameter ', matrixName, ' to ', mat2str(matrixValue), ' for FMU ', fmuName]);
    end
end