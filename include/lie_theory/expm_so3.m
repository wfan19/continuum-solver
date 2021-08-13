function SO3_out = expm_so3(omega)

% Implementation of Rodriguezs' formula
% Reference from the appendix of: https://arxiv.org/pdf/1812.01537.pdf
theta = norm(omega);

if class(theta) == "sym"
    SO3_out = eye(3, class(omega)) + sin(theta)/theta * mat_skew_sym(omega) + ...
            (1 - cos(theta))/theta^2 * mat_skew_sym(omega)^2;
else
    if round(theta, 6) > 0
        SO3_out = eye(3, class(omega)) + sin(theta)/theta * mat_skew_sym(omega) + ...
            (1 - cos(theta))/theta^2 * mat_skew_sym(omega)^2;
    else
        SO3_out = eye(3, class(omega)) + mat_skew_sym(omega) + 0.5*mat_skew_sym(omega)^2;
    end
end


end

