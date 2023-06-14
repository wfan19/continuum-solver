classdef Muscle3D < Muscle
% Implementation of storage, pose calculation, and visualization of a 3D 
% soft-robot muscle parameterized by a "flow-vector": the se(3) body-velocity
% of a frame travelling along the curve of the muscle.

    properties
        % Shearing vector (meters) of muscle
        % Corresponds to the velocity in the body y and z axes of a frame
        % attached to the curve.
        gamma = [0; 0];
    end

    methods
        %% Constructor
        function obj = Muscle3D(l, kappa, options)
            arguments
                l (1, 1) double
                kappa (3, 1) double = zeros(3, 1);
                options.adjoint_X_o = eye(4);
                options.g_0 = eye(4)
                options.color = 'k'
            end
            
            obj@Muscle(l, kappa(:), 'color', options.color)
            
            obj.g_0 = options.g_0;
            obj.adjoint_X_o = options.adjoint_X_o;
        end
        
        
        %% Member functions
        % Calculate position(s) along the curve
        function [g_out, obj] = calc_posns(obj, h_tilde, options)
            arguments
                obj
                h_tilde = obj.h_tilde; % Flow vector
                options.n = 1; % Number of points along the curve for the poses to be calculated at (0 - 1, percentage).
                options.g_offset = eye(4);
            end
            t = linspace(0, 1, options.n);
            
            positions = zeros(length(t), 3);
            orientations = zeros(length(t), 4);

            % Loop through points along the curve (t) and calculate the
            % pose for each point
            for i = 1 : length(t)
                % Calculate and save the transformation for each point              
                SE3_pose_i = options.g_offset * obj.g_0 * se3.expm(t(i) * h_tilde);
                positions(i, :) = SE3_pose_i(1:3, 4);
                orientations(i, :) = rotm2quat(SE3_pose_i(1:3, 1:3));
            end
            
            g_out = table(positions, orientations);
        end
        
        % Plot the current muscle curve on the given line handle. If a
        % line handle is not provided, create a new one on the current axis
        function [lh, g_muscle, obj] = plot_muscle(obj, ax, lh, options)
            arguments
                obj
                ax = gca()
                lh = obj.lh;
                options.resolution = obj.default_res;
                options.line_options = struct() % Additional keyword-arguments for the line options
                options.g_offset = eye(4);
            end
            
            if lh == 0 || (isa(lh, 'matlab.graphics.primitive.Line') && ~isvalid(lh)) % Uninitialized line handle
                lh = line(ax, 0, 0, 0, 'color', obj.color, options.line_options);
                obj.lh = lh;
            end
            
            g_muscle = obj.calc_posns('n', options.resolution, 'g_offset', options.g_offset);
            if ax ~= lh.Parent
                lh.Parent = ax;
            end
            
            lh.XData = g_muscle.positions(:, 1);
            lh.YData = g_muscle.positions(:, 2);
            lh.ZData = g_muscle.positions(:, 3);
        end
        
        % Plot the transformation-frames associated with the flow along the
        % muscle curve, using the plotTransforms function.
        % Returns `gh` - a handle to a Group object which is associated
        % with the transforms plotted.
        function [gh, g_muscle] = plot_tforms(obj, ax, options)
            arguments
                obj
                ax = gca
                options.g_muscle
                options.resolution = obj.default_res;
                options.plot_options = struct(); % Additional keyword-arguments for the plotTransform options
                options.g_offset = eye(4);
            end
            options.plot_options.parent = ax;
            
            g_muscle = obj.calc_posns('n', options.resolution, 'g_offset', options.g_offset);
            plotTransforms(g_muscle.positions, g_muscle.orientations, options.plot_options);
            
            gh = ax.Children(1);
        end
        
        %% Setter function
        % Called in Muscle.set.h_tilde
        function [obj] = update_gamma_kappa(obj, h_tilde)
            % Update gamma if it exists, and safeguard to prevent infitnit
            % looping
            if any(obj.gamma ~= h_tilde(2:3) / h_tilde(1))
                obj.gamma = h_tilde(2:3) / h_tilde(1);
            end

            % Update kappa accordingly
            % Safeguard to prvent infinite looping
            if any(obj.kappa ~= h_tilde(4:end) / h_tilde(1))
                obj.kappa = h_tilde(4:end) / h_tilde(1);
            end
        end
    end
end

