function [tau] = docking_wrapper(t, eta, eta_dot, accel)
    % t: current time

    % Retrieve parameters from base workspace
    controller_type = evalin('base', 'controller_type');
    batch = evalin('base', 'batch');
    run = evalin('base', 'run');
    run_description = evalin('base', 'run_description');
    quay_type = evalin('base', 'quay_type');

    coder.extrinsic('py.importlib.import_module', 'py.str', 'py.float')
   
    % Create instances
    persistent docking_controller;
    dt = 0.1;   % Running docking algorithm on 10 Hz, rest of the simulation on 100 Hz
    %persistent prev_time

    %if isempty(prev_time)
        %dt = t;
        %if_dt = dt
    %else
        %dt = t - prev_time;
        %else_dt = dt
    %end

    if isempty(docking_controller)
        % Import Python module
        %clear docking_code;
        %clear extended_dp_controller;
        %clear prev_time;
        docking_code = py.importlib.import_module('docking_python');
        init_pose = eta;
        init_velocity = eta_dot;
        docking_controller = docking_code.AutoDocking(t, dt, init_pose, init_velocity, batch, run, controller_type, quay_type, run_description);
    end

    %prev_time = t;  % Update previous time for next iteration
    
    % Call the 'run' method from Python with the current simulation time and dt
    tau = double(docking_controller.run(t, dt, eta, eta_dot, accel, controller_type));     % returns control_forces      
end

    
