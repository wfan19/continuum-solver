classdef so3 < gl_n
    %SO3 Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        dof = 6
        mat_size = [3, 3]
    end
    
    methods
        function obj = so3()
        end
    end
    
    methods(Static)
        function mat_omega = hat(v_omega)
            % Create a 3D skew symmetric matrix with a vector v

            mat_omega = zeros(3, 3, class(v_omega));
            mat_omega(2, 1) = v_omega(3);
            mat_omega(3, 1) = -v_omega(2);
            mat_omega(3, 2) = v_omega(1);

            mat_omega = mat_omega + -mat_omega';
        end
        
        function v_omega = vee(mat_omega)
            v_omega = zeros(3, 1);
            v_omega(1) = mat_omega(3, 2);
            v_omega(2) = -mat_omega(3, 1);
            v_omega(3) = mat_omega(2, 1);
        end
        
        function out = expm(v_omega)
            % Check if input was accidentally a matrix
            if all(size(v_omega) == [3, 3])
                v_omega = so3.vee(v_omega);
            end
            % Implementation of Rodriguezs' formula
            % Reference from the appendix of: https://arxiv.org/pdf/1812.01537.pdf
            theta = norm(v_omega);

            if strcmp(class(theta),"sym")
                out = eye(3, class(v_omega)) + sin(theta)/theta * so3.hat(v_omega) + ...
                        (1 - cos(theta))/theta^2 * so3.hat(v_omega)^2;
            else
                if round(theta, 6) > 0
                    out = eye(3, class(v_omega)) + sin(theta)/theta * so3.hat(v_omega) + ...
                        (1 - cos(theta))/theta^2 * so3.hat(v_omega)^2;
                else
                    out = eye(3, class(v_omega)) + so3.hat(v_omega) + 0.5*so3.hat(v_omega)^2;
                end
            end
        end
    end
end

