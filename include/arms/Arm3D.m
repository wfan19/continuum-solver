classdef Arm3D < Arm
    
    properties
        v_lh_circles % Array of ring line handles
        n_circles = 15;
        
        rho = 0.02 % Radius of arm
    end
    
    methods
        %% Constructor
        function obj = Arm3D(g_o, g_muscles, l_0, options)
            arguments
                g_o (4, 4) double = eye(4)
                g_muscles = {eye(4)}
                l_0 = 0.3
                options.plot_unstrained = false
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
                
                ad_o_i = ad_se3(g_o_i); % Adjoint from base curve to muscle_i
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
                mat_M = mat_M + ...
                    muscle_i.adjoint_X_o' * ...
                    diag([1, 1, 1, 0, 0, 0]) * ...
                    muscle_i.adjoint_X_o;
            end
            obj.mat_N = pinv(mat_M) * mat_V;

            %%% Unstrained muscles
            % Create copy of muscles array for future unstrained muscle plotting
            obj.muscles_unstrained = copy(obj.muscles);
            
            % Save the setting
            obj.plot_unstrained = options.plot_unstrained;
        end
        
        %% Plotting initialization
        function obj = initialize_plotting(obj, ax, options)
            arguments
                obj
                ax
                options.line_options_muscles = struct();
                options.line_options_circles = struct();
            end
            % Call superclass plotting initialization
            initialize_plotting@Arm(obj, ax, "line_options_muscles", options.line_options_muscles);
            
            % Label third axis
            zlabel(ax, "Z");

            % Create line handles for circles
            obj.v_lh_circles = matlab.graphics.primitive.Line.empty(0, obj.n_circles);
            for i = 1 : obj.n_circles
                obj.v_lh_circles(i) = line(0, 0, 0, 'color', 'k', options.line_options_circles);
                plot_circle(obj.v_lh_circles(i), obj.rho);
            end
            
            view(ax, 10, 10);
        end
        
        %% Calculate and plot new arm geometry given new length vector
        function h_o_tilde = update_arm(obj, v_l)
            h_o_tilde = update_arm@Arm(obj, v_l);
            
            t_circles = linspace(0, 1, obj.n_circles);
            for i = 1 : length(obj.v_lh_circles)
                g_circle = obj.g_o * expm_se3(h_o_tilde * t_circles(i)) * inv(obj.g_o);
                plot_circle(obj.v_lh_circles(i), obj.rho, g_circle);
            end
        end
    end
end

