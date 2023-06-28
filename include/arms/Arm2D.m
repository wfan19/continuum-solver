classdef Arm2D < Arm
% 2D implementation of a McKibben Arm: A soft continuum robot arm
% constructed of multiple McKibben muscles constrained together by a series
% of constant/identical separators that create motion through contracting
% individual muscles.
%
% See help text for the base class Arm() for more information
    
    properties
        rho = 0
        dimension = 2
    end
    
    methods
        %% Constructor
        function obj = Arm2D(g_o, g_o_muscles, l_0, mat_K, options)
            arguments
                g_o (3, 3) double = eye(3)
                g_o_muscles = {eye(3)}
                l_0 = 0.3
                mat_K = diag([1, 0, 0]);
                options.plot_unstrained = false
            end

            obj.g_o = g_o;
            obj.g_o_muscles = g_o_muscles;

            %%% Create base curve "muscle" for visualization purposes
            obj.muscle_o = Muscle2D(l_0, 0, "adjoint_X_o", eye(3), "g_0", g_o);
            
            %%% Preallocate muscle arrays
            obj.muscles = Muscle2D.empty(0, length(g_o_muscles));
            
            %%% Create individual muscle objects
            hues = linspace(0, 240/360, length(g_o_muscles)); % Colors, gradient from red -> green -> blue
            for i = 1 : length(g_o_muscles)
                g_o_i = g_o_muscles{i};
                g_0_i = g_o * g_o_muscles{i};
                
                ad_o_i = SE2.adjoint(g_o_i); % Adjoint from base curve to muscle_i
                ad_i_o = inv(ad_o_i); % Adjoint from muscle_i to base curve
                
                % Create muscle object
                obj.muscles(i) = Muscle2D(l_0, 0, "adjoint_X_o", ad_i_o, "g_0", g_0_i);
                obj.muscles(i).g_o_i = g_o_i;
                
                % Assign colors
                obj.muscles(i).color = hsv2rgb([hues(i), 1, 1]);    
            end
            
            %%% Compute N matrix
            mat_M = zeros(3);
            mat_V = zeros(3, length(obj.muscles));
            
            for i = 1 : length(obj.muscles)
                v_i = obj.muscles(i).adjoint_X_o' * [1; 0; 0];
                mat_V(:, i) = v_i;
                mat_M = mat_M + ...
                    obj.muscles(i).adjoint_X_o' * ...
                    mat_K * ...
                    obj.muscles(i).adjoint_X_o;
            end
            obj.mat_N = pinv(mat_M) * mat_V;
            
            %%% Unstrained muscles
            % Create copy of muscles array for future unstrained muscle plotting
            obj.muscles_unstrained = copy(obj.muscles);
            
            % Save the setting
            obj.plot_unstrained = options.plot_unstrained;

            %%% Misc
            % Calculate default radius value
            obj.rho = norm(SE2.translation(g_o_muscles{1}));
        end
        
        %% Plotting initialization
        function obj = initialize_plotting(obj, ax, options)
            arguments
                obj
                ax
                options.resolution = 20;
                options.line_options_muscles = obj.line_options_muscles
                options.line_options_spacers = obj.line_options_muscles
                options.line_options_base_curve = obj.line_options_muscles;
            end
            initialize_plotting@Arm(obj, ax, ...
                "line_options_muscles", options.line_options_muscles, ...
                "resolution", options.resolution);
            
            obj.v_lh_spacers = matlab.graphics.primitive.Line.empty(0, obj.n_spacers);
            t_spacers = linspace(0, 1, obj.n_spacers);
            for i = 1 : obj.n_spacers
                obj.v_lh_spacers(i) = line(0, 0, 'color', 'k', options.line_options_spacers);
                
                g_o_i = obj.g_o * se2.expm(t_spacers(i) * obj.muscle_o.h_tilde);
                g_spacer_i = g_o_i * inv(obj.g_o);
                
                plot_spacer(obj.v_lh_spacers(i), obj.rho, g_spacer_i);
            end
            
            % Initialize muscle_o
            obj.muscle_o.plot_muscle(obj.ax, "line_options", options.line_options_base_curve);
            
            if ~ obj.plot_base_curve
                obj.muscle_o.lh.Visible = false;
            end
        end
        
        %% Calculate and plot new arm geometry given new length vector
        function h_o_tilde = update_arm(obj, h_o_tilde, v_l, options)
            arguments
                obj
                % Calculate new base-curve flow-vector if one is not provided
                h_o_tilde
                v_l = -1;
                options.plotting = true;
            end
            update_arm@Arm(obj, h_o_tilde, v_l);
            
            if options.plotting
                obj.plot_arm();
            end
        end

        function plot_arm(obj, ax)
            arguments
                obj
                ax = gca
            end
            plot_arm@Arm(obj, ax);
            obj.muscle_o.lh.Visible = obj.plot_base_curve;

            if ~isempty(obj.v_lh_spacers)
                t_spacers = linspace(0, 1, obj.n_spacers);
                for i = 1 : obj.n_spacers
                    g_o_i = obj.g_o * se2.expm(t_spacers(i) * obj.muscle_o.h_tilde);
                    g_spacer_i = g_o_i * SE2.hat([0, 0, pi/2]);
                    plot_spacer(obj.v_lh_spacers(i), obj.rho, g_spacer_i);
                end
            end
        end
        
    end
end

