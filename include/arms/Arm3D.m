classdef Arm3D < Arm
% 3D implementation of a McKibben Arm: A soft continuum robot arm
% constructed of multiple McKibben muscles constrained together by a series
% of constant/identical separators that create motion through contracting
% individual muscles.
%
% See help text for the base class Arm() for more information
    
    properties
        rho = 0.02 % Radius of arm
        
        gh_base_curve = 0;
        dimension = 3;
    end
    
    methods
        %% Constructor
        function obj = Arm3D(g_o, g_muscles, l_0, options)
            arguments
                g_o (4, 4) double = eye(4)
                g_muscles = {eye(4)}
                l_0 = 0.3
                options.plot_unstrained = false
                options.mat_K = 0;
            end

            %%% Create base curve "muscle" for visualization purposes
            obj.g_o = g_o;
            obj.muscle_o = Muscle3D(l_0, 0, "adjoint_X_o", eye(4), "g_0", g_o);
            
            %%% Preallocate muscle arrays
            obj.muscles = Muscle3D.empty(0, length(g_muscles));
            
            %%% Create individual muscle objects
            g_o_inv = inv(g_o);
            hues = linspace(0, 240/360, length(g_muscles)); % Colors, gradient from red -> green -> blue
            for i = 1 : length(g_muscles)
                g_i = g_muscles{i}; % Muscle pose in world frame
                g_o_i = g_o_inv * g_i; % Tform from base curve to muscle_i
                
                ad_o_i = SE3.adjoint(g_o_i); % Adjoint from base curve to muscle_i
                ad_i_o = inv(ad_o_i); % Adjoint from muscle_i to base curve
                
                % Create muscle object
                obj.muscles(i) = Muscle3D(l_0, 0, "adjoint_X_o", ad_i_o, "g_0", g_i);
                
                % Assign colors
                obj.muscles(i).color = hsv2rgb([hues(i), 1, 1]);    
            end
            
            %%% Compute N matrix
            mat_M = zeros(6);
            mat_V = zeros(6, length(obj.muscles));

            for i = 1 : length(obj.muscles)
                muscle_i = obj.muscles(i);
                mat_V(:, i) = muscle_i.adjoint_X_o' * [1; 0; 0; 0; 0; 0];
                
                % If mat_K optional parameter not set, make a default one
                if size(options.mat_K) == 1
                    k_kappa = 0.00012;
                    v_k = [1 5 5 0 k_kappa k_kappa]';
                    options.mat_K = diag(v_k);
                end
                mat_M = mat_M + ...
                    muscle_i.adjoint_X_o' * ...
                    options.mat_K * ...
                    muscle_i.adjoint_X_o;
            end
            obj.mat_N = pinv(mat_M) * mat_V;

            %%% Unstrained muscles
            % Create copy of muscles array for future unstrained muscle plotting
            obj.muscles_unstrained = copy(obj.muscles);
            
            % Save the setting
            obj.plot_unstrained = options.plot_unstrained;

            %%% Misc
            % Calculate default radius value
            obj.rho = norm(SE3.translation(obj.g_o * inv(g_muscles{1})));
        end
        
        %% Plotting initialization
        function obj = initialize_plotting(obj, ax, options)
            arguments
                obj
                ax
                options.resolution = 20;
                options.line_options_muscles = struct();
                options.line_options_spacers = struct();
            end
            % Call superclass plotting initialization
            initialize_plotting@Arm(obj, ax, "line_options_muscles", options.line_options_muscles, ...
                "resolution", options.resolution);
            
            % Label third axis
            zlabel(ax, "Z");

            % Create line handles for circles
            obj.v_lh_spacers = matlab.graphics.primitive.Line.empty(0, obj.n_spacers);
            
            t_circles = linspace(0, 1, obj.n_spacers);
            for i = 1 : obj.n_spacers
                obj.v_lh_spacers(i) = line(0, 0, 0, 'color', 'k', options.line_options_spacers);
                
                g_circle = obj.g_o * se3.expm(obj.muscle_o.h_tilde * t_circles(i)) * inv(obj.g_o);
                plot_circle(obj.v_lh_spacers(i), obj.rho, g_circle);
            end
            
            view(ax, 10, 10);
        end
        
        %% Calculate and plot new arm geometry given new length vector
        function h_o_tilde = update_arm(obj, v_l, h_o_tilde)
            arguments
                obj
                v_l
                % Calculate new base-curve flow-vector if one is not provided
                h_o_tilde = obj.f_h_o_tilde(obj, v_l);
            end
            update_arm@Arm(obj, v_l, h_o_tilde);
            
            t_circles = linspace(0, 1, obj.n_spacers);
            for i = 1 : length(obj.v_lh_spacers)
                g_muscle_circle = SE3.hat(eul2rotm([0, pi/2, 0], 'xyz'), [0; 0; 0]);
                g_circle = obj.g_o * se3.expm(h_o_tilde * t_circles(i)) * g_muscle_circle;
                plot_circle(obj.v_lh_spacers(i), obj.rho, g_circle);
            end
            
            if class(obj.gh_base_curve) ~= "double"
                delete(obj.gh_base_curve);
                obj.gh_base_curve = 0;
            end
            
            if obj.plot_base_curve
                plot_options = struct("FrameSize", obj.rho * 0.5);
                obj.gh_base_curve = obj.muscle_o.plot_tforms(obj.ax, "plot_options", plot_options);
            end
        end
    end
    
    methods(Static)
        % Generate muscle poses in a circle on the Y-Z plane around a given
        % base-pose
        function g_muscles = generate_g_muscles(rho, g_o, n_muscles, tilt_angle)
            theta = linspace(0, 2*pi, n_muscles + 1);
            theta = theta(1:end-1);
            
            % Yaw of each muscle
            yaw_muscles = theta - pi/2;
            
            % Base rotation matrix for all muscles (tilt angle)
            % Tilt angle is defined as the angle relative to the base-curve
            % frame's x axis
            R_muscles = eul2rotm([0 0 -tilt_angle], 'xyz');
            
            g_muscles = cell(1, n_muscles);
            for i = 1 : n_muscles
                t_muscle_i = rho * [0; cos(theta(i)); sin(theta(i))];
                R_muscle_i = eul2rotm([yaw_muscles(i), 0, 0], 'xyz') * R_muscles;
                g_muscles{i} = g_o * SE3.hat(R_muscle_i, t_muscle_i);
            end
        end
    end
end

