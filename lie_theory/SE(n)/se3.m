function se3_out = se3(omega, v)
if(nargin == 1)
    % Extract linear/angular components from a single vector
    v = omega(1:3);
    omega = omega(4:6);
end
    
se3_out = zeros(4, class([omega, v]));

mat_omega = mat_skew_sym(omega);

se3_out(1:3, 1:3) = mat_omega;
se3_out(1:3, 4) = v;

end

