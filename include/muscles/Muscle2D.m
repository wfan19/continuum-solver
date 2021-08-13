classdef Muscle2D < Muscle
% Implementatino of storage, pose calculation, and visualization of a 2D 
% soft-robot muscle parameterized by a "flow-vector": the se(2) body-velocity
% of a frame travelling along the curve of the muscle.

    properties
        % Shearing (meters) of muscle
        gamma = 0;
    end

    %% Methods
    methods
        %% Constructor
        function obj = Muscle2D(l, kappa, options)
            arguments
                l (1, 1) double
                kappa (1, 1) double = 0;
                options.adjoint_X_o = eye(3);
                options.g_0 = eye(3)
                options.color = 'k'
            end
            
            obj = obj@Muscle(l, kappa, 'color', options.color);
            
            obj.g_0 = options.g_0;
            obj.adjoint_X_o = options.adjoint_X_o;
        end
        
        %% Member functions
        % Calculate position(s) along the curve
        function [g_out, obj] = calc_posns(obj, h_tilde, options)
            arguments
                obj
                h_tilde = obj.h_tilde; % Flow vector
                options.t = 1; % Array of points along the curve for the poses to be calculated at (0 - 1, percentage).
            end
            
            % Update object's flow-vector if the input flow-vector isn't
            % the one currently stored
            if h_tilde ~= obj.h_tilde
                obj.h_tilde = h_tilde;
            end
            
            g_out = zeros(3, length(options.t));

            % Loop through points along the curve (t) and calculate the
            % pose for each point
            for i = 1 : length(options.t)
                % Calculate and save the transformation for each point
                SE2_pose_i = obj.g_0 * expm_se2(options.t(i) * obj.h_tilde);
                g_out(:, i) = [SE2_pose_i(1, 3); SE2_pose_i(2, 3); atan2(SE2_pose_i(2, 1), SE2_pose_i(1, 1))];
            end
        end
        
        % Plot the current muscle curve on the given line handle. If a
        % line handle is not provided, create a new one on the current axis
        function [lh, obj] = plot_muscle(obj, ax, lh, options)
            arguments
                obj
                ax = gca()
                lh = obj.lh;
                options.resolution = obj.default_res;
                options.line_options = struct() % Additional keyword-arguments for the line options
            end
            
            if lh == 0 % Uninitialized line handle
                lh = line(ax, 0, 0, 'color', obj.color, options.line_options);
                obj.lh = lh;
            end
            
            g_muscle = obj.calc_posns('t', linspace(0, 1, options.resolution));
            if ax ~= lh.Parent
                lh.Parent = ax;
            end
            
            lh.XData = g_muscle(1, :);
            lh.YData = g_muscle(2, :);
        end
        
        %% Setter functions
        % Called in Muscle.set.h_tilde
        function [obj] = update_gamma_kappa(obj, h_tilde)
            % Update gamma if it exists, and safeguard to prevent infitnit
            % looping
            if any(obj.gamma ~= h_tilde(2) / h_tilde(1))
                obj.gamma = h_tilde(2) / h_tilde(1);
            end

            % Update kappa accordingly
            % Safeguard to prvent infinite looping
            if any(obj.kappa ~= h_tilde(3) / h_tilde(1))
                obj.kappa = h_tilde(3) / h_tilde(1);
            end
        end
    end
end

