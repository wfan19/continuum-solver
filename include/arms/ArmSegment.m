classdef ArmSegment < handle & matlab.mixin.Copyable
    % A constant cross-section arm segment.
    
    properties
        group 
        rod_o
        rods
    end
    
    methods
        function obj = ArmSegment(group, g_o, g_o_rods, l)
            obj.group = group;

            % Create the base-curve
            obj.rod_o = Rod(group, l, g_o); % TODO: Handle different muscle lengths?
            obj.rods = Rod.empty(0, length(g_o_rods));

            for i = 1 : length(g_o_rods)
                g_0_i = g_o * g_o_rods{i};
                obj.rods(i) = Rod(group, l, g_0_i);
            end
        end

        function strains = get_strains(obj)

        end

        function forces = get_forces(obj)

        end

        function pose = get_tip_pose(obj)
            pose = obj.group.hat(obj.rod_o.calc_posns());
        end

        function ax = plot()
            % TODO: Does this get refactored out?
        end
    end

    methods(Access = protected)
        function cp = copyElement(obj)
            cp = copyElement@matlab.mixin.Copyable(obj);

            % Deepcopy all the rod objects
            for i = 1 : length(obj.rods)
                cp.rods(i) = copy(obj.rods(i));
            end
            cp.rod_o = copy(obj.rod_o);
        end
    end
end

