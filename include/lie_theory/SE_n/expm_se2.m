function SE2_out = expm_se2(vx, vy, omega)
    % Analytic SE2 exponential map
    % - Supports both numerical and symbolic calculations
    % - If any input contains a symbolic object it will return a symbolic
    % matrix
    
    if nargin == 1
        % Single flow-vector input
        v = vx;
        vx = v(1);
        vy = v(2);
        omega = v(3);
    end

    skew_sym_2 = [0 -1; 1 0];
    if omega ~= 0
        V = (1/omega) * (sin(omega)*eye(2) + ((1-cos(omega))*skew_sym_2));
    else
        V = eye(2, class([vx, vy, omega]));
    end
    
    R = SO2(omega);
    
    SE2_out = eye(3, class([vx, vy, omega]));
    SE2_out(1:2, 1:2) = R;
    SE2_out(1:2, 3) = V * [vx; vy];
end