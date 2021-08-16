classdef Arm2D < handle
    %ARM2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        muscles % Array of muscle objects
        muscles_unstrained % Array of muscle objects (for plotting unstrained muscles)
        
        muscle_o % Base muscle
        
        mat_N
        
        gh_tform
        
        ax % Plotting axes
        
        line_options_muscles = struct()
        
        plot_unstrained = false
        
        % Base curve flow-vector calculation function handle
        f_h_o_tilde = @f_h_o_tilde_default
    end
    
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
                ax = gca;
                options.line_options_muscles = obj.line_options_muscles
            end
            obj.ax = ax;
            
            axis(ax, 'equal');
            grid(ax, 'on');
            xlabel(ax, "X");
            ylabel(ax, "Y");
            
            for i = 1 : length(obj.muscles)
                % Clear existing line handle
                obj.muscles(i).lh = 0;
                
                % Create new line handles
                obj.muscles(i).plot_muscle(ax, 'line_options', options.line_options_muscles);
            end 
            
            % Initialize unstrained muscles
            obj.muscles_unstrained = copy(obj.muscles);
            for i = 1 : length(obj.muscles_unstrained)
                muscle_color_hsv = rgb2hsv(obj.muscles(i).color);
                muscle_unstrained_color = hsv2rgb([1, 1, 0.5] .* muscle_color_hsv); % Same color but darker
                obj.muscles_unstrained(i).color = muscle_unstrained_color;
                obj.muscles_unstrained(i).lh.Color = muscle_unstrained_color;
                obj.muscles_unstrained(i).lh.LineStyle = ':';
                obj.muscles_unstrained(i).plot_muscle(ax);
            end
            
            % TODO: Add dividers?
        end

        %% Recalculate geometry upon updating muscle lengths
        % Update arm geometry for a new length-vector - a vector with
        % individual muscle lengths
        function update_arm(obj, v_l)
            %%% Calculate new base-curve flow-vector
            h_o_tilde = obj.f_h_o_tilde(obj, v_l);
            
            %%% Calculate and plot individual muscle lengths
            for i = 1 : length(obj.muscles)
                obj.muscles(i).h_tilde = obj.muscles(i).adjoint_X_o * h_o_tilde;

                % Plot muscles
                obj.muscles(i).plot_muscle(obj.ax, 'line_options', obj.line_options_muscles);
                
                %%% Plot unstrained muscles
                if obj.plot_unstrained
                    % Update unstrained muscle's curvature
                    obj.muscles_unstrained(i).h_tilde = obj.muscles(i).h_tilde;
                    % Set unstrained muscle's length back to the input length
                    obj.muscles_unstrained(i).l = v_l(i);
                    
                    % Plot unstrained muscles and set visibile
                    obj.muscles_unstrained(i).plot_muscle(obj.ax);
                    obj.muscles_unstrained(i).lh.Visible = true;
                else
                    % Hide unstrained muscles
                    obj.muscles_unstrained(i).lh.Visible = false;
                end
            end
        end
        
        % Default base-curve flow-vector calculation function
        % Implementation of least-squares (assumes linear material)
        % The current 2D implementation also assumes 0 base-curve shearing
        % and the base curve shearing is impossible
        function h_o_tilde = f_h_o_tilde_default(obj, v_l)
            h_o_tilde = obj.mat_N * v_l;
        end
    end
end

