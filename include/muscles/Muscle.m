classdef Muscle < handle & matlab.mixin.Copyable
% Parent class for Muscle objects
% - Handles general properties 
% - Implements custom Setter functions (linking configuration <-> flow-vector)
% - Copy constructor

    %% Properties
    properties
        % Transform from base curve's frame to muscle base's frame (x = forward)
        g_0
        
        % Adjoint matrix from self to base-curve
        adjoint_X_o
        
        % Length (meters) of muscle
        l = 0.1
        
        % Curvature of muscle
        kappa
        
        % Flow vector of muscle
        h_tilde
        
        % Plotting resolution
        default_res = 20;
        
        color = 'k'
        
        % Line handle attached to muscle
        lh = 0 % Default value for uninitialized linehandle
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = Muscle(l, kappa, options)
            arguments
                l
                kappa
                options.color = 'k'
            end
            
            obj.kappa = kappa;
            obj.l = l;
            obj.color = options.color;
        end
        
        %% Setters
        % Here we implement custom setter functions to link the h_tilde
        % property with the l and kappa properties, such that updating one
        % updates the others.
        
        % Setters for l and kappa 
        function set.l(obj, l)
            assert(l ~= 0, "Length of rod cannot be zero")
            obj.l = l;
            % Update h_tilde accordingly
            obj.h_tilde = obj.l * [1; obj.gamma; obj.kappa];
        end
        
        function set.kappa(obj, kappa)
            obj.kappa = kappa;
            % Update h_tilde accordingly
            obj.h_tilde = obj.l * [1; obj.gamma; obj.kappa];
        end
        
        % Function that updates kappa and l values based on an updated
        % h_tilde value
        function set.h_tilde(obj, h_tilde)
            assert(h_tilde(1) ~= 0, "Length in h_tilde cannot be zero")
            obj.h_tilde = h_tilde;
            
            % Update l
            % Safeguard to prevent infinite looping
            if obj.l ~= h_tilde(1)
                obj.l = h_tilde(1);
            end
            
            % Update gamma and kappa (implementation specific)
            obj.update_gamma_kappa(h_tilde);

        end
        
        function set.color(obj, color)
            obj.color = color;
            if class(obj.lh) ~= "double" % If line handle has been initialized
                obj.lh.Color = color;
            end
        end
        
        %% Member functions
        % To be implemented in subclasses
        
        % Calculate position(s) along the curve
        function [g_out, obj] = calc_posns(obj, h_tilde, options)
        end
        
        % Plot the current muscle curve on the given line handle. If a
        % line handle is not provided, create a new one on the current axis
        function [lh, obj] = plot_muscle(obj, ax, lh, options)
        end
    end
    
    % Copy constructor
    methods (Access = protected)
        
        % Copy constructor
        % Inherited from matlab.mixin.Copyable
        function cp = copyElement(obj)
            % Regular copy of all elements
            cp = copyElement@matlab.mixin.Copyable(obj);
            
            % Copy value of line-handle if initialized
            if obj.lh ~= 0
                cp.lh = copy(obj.lh);
                cp.lh.Parent = obj.lh.Parent;
            end
        end
    end
end

