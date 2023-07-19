classdef Muscle < handle & matlab.mixin.Copyable
% Parent class for Muscle objects
% - Handles general properties 
% - Implements custom Setter functions (linking configuration <-> flow-vector)
% - Copy constructor

    %% Properties
    properties
        group           % Embedding Lie group

        g_0             % Pose of muscle in world frame
        max_s = 1       % Default s bound
        
        % Flow vector storage: components vs whole
        g_circ_right    % Flow vector of muscle
        length = 0.1    % Length of muscle
        true_shear           % Shear of muscle
        true_curvature       % Curvature of muscle

        % Components - TODO: Should these be mixins instead?
        mechanics       % Mechanics model
        plotter         % Plotting module: 2D or 3D
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = Muscle(length, curvature)
            obj.true_curvature = curvature;
            obj.length = length;
        end

        %% Member functions
        % Calculate position(s) along the curve
        function [g_out, obj] = calc_posns(obj, g_circ_right, options)
            obj.group
        end
        
        %% Setters
        % Here we implement custom setter functions to link the h_tilde
        % property with the l and kappa properties, such that updating one
        % updates the others.
        
        % Setters for l and kappa 
        function set.length(obj, length)
            assert(length ~= 0, "Length of rod cannot be zero")
            obj.length = length;
            % Update h_tilde accordingly
            obj.g_circ_right = obj.length * [1; obj.true_shear; obj.true_curvature];
        end
        
        function set.true_curvature(obj, curvature)
            obj.true_curvature = curvature;
            % Update h_tilde accordingly
            obj.g_circ_right = obj.length * [1; obj.true_shear; obj.true_curvature];
        end
        
        % Function that updates kappa and l values based on an updated
        % h_tilde value
        function set.g_circ_right(obj, g_circ_right)
            assert(g_circ_right(1) ~= 0, "Length in g_circ_right cannot be zero") % THis breaks the safeguards
            obj.g_circ_right = g_circ_right;
            
            % Update l
            new_l = g_circ_right(1);
            if obj.length ~= new_l  % Only set new val if not already set to prevent infinite loop
                obj.length = new_l;
            end

            v = obj.group.algebra.v_translation(g_circ_right);
            omega = obj.group.algera.v_rotation(g_circ_right);

            new_shear = v(2:end) / new_l;
            new_curvature = omega / new_l;

            % Only set new val if not already set to prevent infinite loop
            if any(obj.true_shear ~= new_shear) 
                obj.true_shear = new_shear;
            end
            
            if any(obj.true_curvature ~= new_curvature)
                obj.true_curvature = new_curvature;
            end
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
            if obj.lh ~= 0 && isvalid(obj.lh)
                cp.lh = copy(obj.lh);
                cp.lh.Parent = obj.lh.Parent;
            end
        end
    end
end

