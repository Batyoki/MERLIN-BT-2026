function result = run_control_sim(control_law_text, param_names, param_values, sim_time, initial_state, ref_signal, ref_amplitude)
% RUN_CONTROL_SIM Main simulation function for control algorithms
%
% Inputs:
%   control_law_text - Free-text control law that assigns to u
%   param_names - Cell array of parameter names used by the control law
%   param_values - Array of parameter values corresponding to param_names
%   sim_time - Simulation duration (seconds)
%   initial_state - Initial system state (empty for default)
%   ref_signal - Reference signal type ('step', 'ramp', 'sine')
%   ref_amplitude - Amplitude of reference signal
%
% Output:
%   result - Struct with simulation results

    % Time vector
    dt = 0.01;  % 10ms sampling time
    time = 0:dt:sim_time;
    N = length(time);
    
    % System definition (simple second-order mass-spring-damper)
    % State: [position; velocity]
    % x_dot = Ax + Bu
    % y = Cx + Du
    m = 1.0;    % mass (kg)
    b = 0.5;    % damping (N-s/m)
    k = 2.0;    % spring constant (N/m)
    
    A = [0, 1; -k/m, -b/m];
    B = [0; 1/m];
    C = [1, 0];
    D = 0;
    
    % Initial state
    if isempty(initial_state)
        x = [0; 0];  % Start at rest
    else
        x = initial_state(:);
    end
    
    % Generate reference signal
    reference = generate_reference(time, ref_signal, ref_amplitude);
    
    % Pre-allocate arrays
    states = zeros(2, N);
    outputs = zeros(1, N);
    controls = zeros(1, N);
    errors = zeros(1, N);
    
    % Controller state (if needed)
    controller_state = [];
    integral_e = 0;
    prev_e = 0;
    
    % Simulation loop
    for i = 1:N
        % Store current state
        states(:, i) = x;
        y = C * x + D * 0;
        outputs(i) = y;
        
        % Compute error
        r = reference(i);
        e = r - y;
        errors(i) = e;

        integral_e = integral_e + e * dt;
        derivative_e = (e - prev_e) / dt;

        if isempty(control_law_text)
            u = 0;
        else
            u = eval_custom_law(control_law_text, param_names, param_values, e, dt, integral_e, derivative_e, x, r, time(i));
        end
        
        % Saturate control (actuator limits)
        u_max = 10.0;
        u = max(min(u, u_max), -u_max);
        controls(i) = u;
        
        prev_e = e;

        % System dynamics (Euler integration)
        x_dot = A * x + B * u;
        x = x + x_dot * dt;
    end
    
    % Package results
    result = struct();
    result.time = time;
    result.states = outputs;  % Return output (position) as main state
    result.control = controls;
    result.reference = reference;
    result.error = errors;
    result.full_states = states;  % Include all internal states
end


function ref = generate_reference(time, signal_type, amplitude)
% GENERATE_REFERENCE Create reference signal
    
    switch lower(signal_type)
        case 'step'
            ref = amplitude * ones(size(time));
            
        case 'ramp'
            ref = amplitude * time / max(time);
            
        case 'sine'
            freq = 0.5;  % Hz
            ref = amplitude * sin(2 * pi * freq * time);
            
        case 'square'
            freq = 0.2;  % Hz
            ref = amplitude * (2 * (sin(2 * pi * freq * time) >= 0) - 1);   % square wave without the Signal Processing Toolbox
            
        otherwise
            ref = amplitude * ones(size(time));
    end
end
