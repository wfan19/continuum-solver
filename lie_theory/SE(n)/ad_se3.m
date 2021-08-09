function ad_out = ad_se3(R, t)
% Construct Adjoint matrix for SE3 based on the analytic formula
% Reference: https://arxiv.org/pdf/1812.01537.pdf

% The adjoint matrix maps an element from a non-identity tangent space to
% the equivalent element in the tangent space about the identity (the Lie
% algebra)

if nargin == 1
    % Extract Rotation matrix and translation vector from single pose matrix input
    g = R;
    R = g(1:3, 1:3);
    t = g(1:3, 4);
end

ad_out = blkdiag(R, R);
ad_out(1:3, 4:6) = mat_skew_sym(t) * R;

end

