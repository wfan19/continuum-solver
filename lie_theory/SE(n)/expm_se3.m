function SE3_out = expm_se3(omega, vel)
% Analytic form of the SE3 exponentail map, from here: https://arxiv.org/pdf/1812.01537.pdf
if(nargin == 1)
    % Extract linear/angular components from a single vector
    vel = omega(1:3);
    omega = omega(4:6);
end

class_string = class([omega(:), vel(:)]);
SE3_out = eye(4, class_string);

theta = norm(omega);
if round(theta, 6) > 0
    V = eye(3, class_string) + (1 - cos(theta)) / theta^2 * mat_skew_sym(omega) + ...
        (theta - sin(theta))/theta^3 * mat_skew_sym(omega)^2;
else
    % For small angles this is what the coefficients converge to, not
    % infinity
    % Can be calculated via l'Hopitals
    V = eye(3, class_string) + 0.5 * mat_skew_sym(omega) + (1/6) * mat_skew_sym(omega);
end
    SE3_out(1:3, 1:3) = expm_so3(omega);
    SE3_out(1:3, 4) = V * vel;

end

