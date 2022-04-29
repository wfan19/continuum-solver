classdef Arm < handle & matlab.mixin.Copyable
% Base class for modelling a soft continuum arm constructed of multiple
% McKibben muscles constrained together by a series of constant/identical
% separator pieces. 
% s
% Stores the associated Muscle objects that control the overall shape of
% the McKibben Arm itself, as well as the matrices governing the model 
% Least Squares problem whose solution determins the forward/inverse
% kinematics of the arm. 
%
% Allows for easy initialization, modelling, and plotting of 2D or 3D
% McKibben Arms
    
    properties        
        muscles % Array of muscle objects
        muscles_unstrained % Array of muscle objects (for plotting unstrained muscles)
        
        g_o % Base frame transformation
        muscle_o % Base muscle
        
        mat_N
        
        ax = 0 % Plotting axes
        
        line_options_muscles = struct()
        
        plot_unstrained = false
        
        % Base curve flow-vector calculation function handle
        f_h_o_tilde = @f_h_o_tilde_default
        
        plot_base_curve = false;
        v_lh_spacers
        n_spacers = 4
    end

    properties (Abstract)
        dimension
        rho
    end
    
    methods
        % Constructor
        function obj = Arm()
        end
        
        function initialize_plotting(obj, ax, options)
            arguments
                obj
                ax
                options.resolution = 20
                options.line_options_muscles = obj.line_options_muscles
            end
            
            obj.ax = ax;
            
            axis(ax, 'equal');
            grid(ax, 'on');
            xlabel(ax, "X");
            ylabel(ax, "Y");
            
            for i = 1 : length(obj.muscles)
                obj.muscles(i).default_res = options.resolution;
                
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
            
                if ~obj.plot_unstrained
                    obj.muscles_unstrained(i).lh.Visible = false;
                end
            end
        end
        
        %% Recalculate geometry upon updating muscle lengths
        % Update arm geometry for a new length-vector - a vector with
        % individual muscle lengths
        % * Generic for both 2D and 3D
        function h_o_tilde = update_arm(obj, v_l, h_o_tilde)
            arguments
                obj
                v_l
                % Calculate new base-curve flow-vector if one is not provided
                h_o_tilde = obj.f_h_o_tilde(obj, v_l);
            end
            
            obj.muscle_o.h_tilde = h_o_tilde;
            
            %%% Calculate and plot individual muscle lengths
            for i = 1 : length(obj.muscles)
                obj.muscles(i).h_tilde = obj.muscles(i).adjoint_X_o * h_o_tilde;

                % Plot muscles
                obj.muscles(i).plot_muscle(obj.ax);
                
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
        
        %% Default base-curve flow-vector calculation function
        % Implementation of least-squares (assumes linear material)
        % The current 2D implementation also assumes 0 base-curve shearing
        % and the base curve shearing is impossible
        % * Generic for both 2D and 3D
        function h_o_tilde = f_h_o_tilde_default(obj, v_l)
            h_o_tilde = obj.mat_N * v_l;
        end
    end
    
    % Copy constructor
    methods (Access = protected)
        
        % Copy constructor
        % Inherited from matlab.mixin.Copyable
        function cp = copyElement(obj)
            % Regular copy of all elements
            cp = copyElement@matlab.mixin.Copyable(obj);
            
            % Apply copy cstor for all handle children objects
            cp.muscles = copy(obj.muscles);
            cp.muscle_o = copy(obj.muscle_o);
            cp.muscles_unstrained = copy(obj.muscles_unstrained);
            
            if ~isempty(obj.v_lh_spacers)
                cp.v_lh_spacers = copy(obj.v_lh_spacers);
            end
        end
    end
end
