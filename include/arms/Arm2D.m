classdef Arm2D < Arm
    %ARM2D Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        %% Constructor
        function obj = Arm2D(g_o, g_muscles, l_0, options)
            arguments
                g_o (3, 3) double = eye(3)
                g_muscles = {eye(3)}
                l_0 = 0.3
                options.plot_unstrained = false
            end
            
            %%% Create base curve "muscle" for visualization purposes
            obj.muscle_o = Muscle2D(l_0, 0, "adjoint_X_o", eye(3), "g_0", g_o);
            
            %%% Preallocate muscle arrays
            obj.muscles = Muscle2D.empty(0, length(g_muscles));
            
            %%% Create individual muscle objects
            g_o_inv = inv(g_o);
            hues = linspace(0, 240/360, length(g_muscles)); % Colors, gradient from red -> green -> blue
            for i = 1 : length(g_muscles)
                g_i = g_muscles{i}; % Muscle pose in world frame
                g_o_i = g_o_inv * g_i; % Tform from base curve to muscle_i
                
                ad_o_i = ad_se2(g_o_i); % Adjoint from base curve to muscle_i
                ad_i_o = inv(ad_o_i); % Adjoint from muscle_i to base curve
                
                % Create muscle object
                obj.muscles(i) = Muscle2D(l_0, 0, "adjoint_X_o", ad_i_o, "g_0", g_i);
                
                % Assign colors
                obj.muscles(i).color = hsv2rgb([hues(i), 1, 1]);    
            end
            
            %%% Compute N matrix
            mat_M = zeros(3);
            mat_V = zeros(3, length(obj.muscles));
            
            for i = 1 : length(obj.muscles)
                v_i = obj.muscles(i).adjoint_X_o' * [1; 0; 0];
                mat_V(:, i) = v_i;
                mat_M = mat_M + v_i * v_i';
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
                options.line_options_muscles = obj.line_options_muscles
            end
            initialize_plotting@Arm(obj, ax, "line_options_muscles", options.line_options_muscles);
            % TODO: Add dividers?
        end
    end
end

