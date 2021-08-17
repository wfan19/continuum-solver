classdef Arm < handle
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
        % Constructor
        function obj = Arm(inputArg1,inputArg2)
        end
        
        %% Recalculate geometry upon updating muscle lengths
        % Update arm geometry for a new length-vector - a vector with
        % individual muscle lengths
        % * Generic for both 2D and 3D
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
        
        %%% Default base-curve flow-vector calculation function
        % Implementation of least-squares (assumes linear material)
        % The current 2D implementation also assumes 0 base-curve shearing
        % and the base curve shearing is impossible
        % * Generic for both 2D and 3D
        function h_o_tilde = f_h_o_tilde_default(obj, v_l)
            h_o_tilde = obj.mat_N * v_l;
        end
        
        function initialize_plotting(obj, ax, options)
            arguments
                obj
                ax
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
                obj.muscles(i).plot_muscle(obj.ax, 'line_options', options.line_options_muscles);
            end 
            
            % Initialize unstrained muscles
            obj.muscles_unstrained = copy(obj.muscles);
            for i = 1 : length(obj.muscles_unstrained)
                muscle_color_hsv = rgb2hsv(obj.muscles(i).color);
                muscle_unstrained_color = hsv2rgb([1, 1, 0.5] .* muscle_color_hsv); % Same color but darker
                obj.muscles_unstrained(i).color = muscle_unstrained_color;
                obj.muscles_unstrained(i).lh.Color = muscle_unstrained_color;
                obj.muscles_unstrained(i).lh.LineStyle = ':';
                obj.muscles_unstrained(i).plot_muscle(obj.ax);
            end
            
        end
        
    end
end
