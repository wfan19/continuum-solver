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
            
            %%% Calculate the helix parameters
            v = h_o_tilde(1:3);     % Linear velocity
            omega = h_o_tilde(4:6); % Angular velocity
            
            % Calculate helix parameters in the base-curve frame
            obj.winding_axis_o = omega / norm(omega) * sign(omega(1));
            r_o = cross(omega, v) / norm(omega)^2; % Radius vector
            
            % Calculate transformation from base curve to helix in base curve frame
            R_axis_o = [obj.winding_axis_o, ...
                        -r_o/norm(r_o), ...
                        cross(obj.winding_axis_o, -r_o/norm(r_o))];
            g_o_spiral = SE3(R_axis_o, r_o);
            
            obj.radius_vector_o = r_o;
            v_x_prime = omega(1) * v(1) / norm(omega); % Helix linear velocity in winding axis direction
            obj.pitch = v_x_prime / norm(omega); % Pitch = length / turns

            % Offset to transform the helix to g_o
            % It's really g_A' = g_o * inv(g_o_spiral) * exp(h_A)
            % But plot_muscles has a built in g_o*exp(h_A) so we need to cancel out
            % that g_o
            g_offset = obj.g_o * inv(g_o_spiral) * inv(obj.g_o);
            
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
        end
    end
end

