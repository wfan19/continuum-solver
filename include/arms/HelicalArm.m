classdef HelicalArm < Arm3D
    
    properties
        winding_axis_o
        radius_vector_o
        pitch
    end
    
    methods
        function obj = HelicalArm(g_o, g_muscles, l_0)
            obj = obj@Arm3D(g_o, g_muscles, l_0);
        end
        
        %% Calculate and plot new arm geometry given new length vector
        % Overriden here to keep the helix vertical
        function h_o_tilde = update_arm(obj, v_l, h_o_tilde)
            arguments
                obj
                v_l
                % Calculate new base-curve flow-vector if one is not provided
                h_o_tilde = obj.f_h_o_tilde(obj, v_l);
            end
            
            obj.muscle_o.h_tilde = h_o_tilde;
            
            %%% Calculate the helix parameters
            v = h_o_tilde(1:3);     % Linear velocity
            omega = h_o_tilde(4:6); % Angular velocity
            
            % Calculate helix parameters in the base-curve frame
            obj.winding_axis_o = omega / norm(omega) * sign(omega(1));
            r_o = cross(omega, v) / norm(omega)^2; % Radius vector
            
            %%% Calculate transformation from base curve to helix in base curve frame
            %{ 
            % Math that causes the rotation bug
            % Rotation matrix with winding axis as X axis, and
            radius-vector as Y axis
            R_axis_o = [obj.winding_axis_o, ...
                        r_o / norm(r_o) ...
                        cross(obj.winding_axis_o, r_o / norm(r_o)];
            %}
            
            % Math that does not cause the rotation bug
            % Computes the minimal quaternion that rotates the winding axis
            % to the X axis (base-curve frame), using the bi-normal vector
            % (normalized cross prduct) of the two as the axis of rotation. 
            
            % Compute doubled rotation between the two
            % This rotation is on the correct axis but has doubled the
            % angle due to the theta/2 term in the quaternion axis-angle
            % formulation
            quat_axis_o = [dot(obj.winding_axis_o, [1; 0; 0]); ...
                cross([1; 0; 0], obj.winding_axis_o)];
            
            % Find the half-way rotation (the actual rotation) by summing
            % with the identity rotation ([1; 0; 0; 0]), and then
            % normalizing
            quat_axis_o = quat_axis_o + [1; 0; 0; 0];
            quat_axis_o = quat_axis_o / norm(quat_axis_o);
            
            % Construct the rotation matrix given quaternion
            R_axis_o = quat2rotm(quat_axis_o');
            
            % Transformation matrix of the helix
            g_o_helix = SE3(R_axis_o, r_o);

            %%% Offset to transform the helix to g_o
            % It's really g_A' = g_o * inv(g_o_spiral) * exp(h_A)
            % But plot_muscles has a built in g_o*exp(h_A) so we need to cancel out
            % that g_o
            g_offset = obj.g_o * inv(g_o_helix) * inv(obj.g_o);
            
            %%% Save the helix parameters
            obj.radius_vector_o = r_o;
            v_x_prime = omega(1) * v(1) / norm(omega); % Helix linear velocity in winding axis direction
            obj.pitch = v_x_prime / norm(omega); % Pitch = length / turns
            
            %%% Calculate and plot individual muscle lengths
            for i = 1 : length(obj.muscles)
                obj.muscles(i).h_tilde = obj.muscles(i).adjoint_X_o * h_o_tilde;

                % Plot muscles
                obj.muscles(i).plot_muscle(obj.ax, "g_offset", g_offset);
                
                %%% Plot unstrained muscles
                if obj.plot_unstrained
                    % Update unstrained muscle's curvature
                    obj.muscles_unstrained(i).h_tilde = obj.muscles(i).h_tilde;
                    % Set unstrained muscle's length back to the input length
                    obj.muscles_unstrained(i).l = v_l(i);
                    
                    % Plot unstrained muscles and set visibile
                    obj.muscles_unstrained(i).plot_muscle(obj.ax, "g_offset", g_offset);
                    obj.muscles_unstrained(i).lh.Visible = true;
                else
                    % Hide unstrained muscles
                    obj.muscles_unstrained(i).lh.Visible = false;
                end
            end
            
            %%% Plot circles along the arm
            t_circles = linspace(0, 1, obj.n_circles);
            for i = 1 : length(obj.v_lh_circles)
                g_circle = g_offset * obj.g_o * expm_se3(h_o_tilde * t_circles(i)) * inv(obj.g_o);
                plot_circle(obj.v_lh_circles(i), obj.rho, g_circle);
            end
            
            %%% Plot base curve
            if class(obj.gh_base_curve) ~= "double"
                delete(obj.gh_base_curve);
                obj.gh_base_curve = 0;
            end
            
            if obj.plot_base_curve
                plot_options = struct("FrameSize", obj.rho * 0.5);
                obj.gh_base_curve = obj.muscle_o.plot_tforms(obj.ax, ...
                    "plot_options", plot_options, ...
                    "g_offset", g_offset);
            end
        end
    end
end

