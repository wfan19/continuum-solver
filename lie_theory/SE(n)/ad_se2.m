% Create the analytic adjoint matrix for a given SE2 pose
% Analytic SE2 adjoint formula from https://arxiv.org/pdf/1812.01537.pdf
function adj_out = ad_se2(theta, x, y)
% This function is a stinking pile of poop I am sorry that you have to read
% this

if nargin == 1 % Single input
    arg1 = theta;
    if numel(arg1) == 9
        % Pose matrix input
        t = arg1(1:2, 3);
    else
        % Transform vector input
        % Vector: [x, y, theta]
        t = arg1(1:2);
        theta = arg1(3);
    end
elseif nargin == 2 % Two input: position vector, scalar angle
    t = x;
else % Three inputs: Reconstruct position vector
    t = [x; y];
end

if numel(arg1) == 9
    R = arg1(1:2, 1:2);
else
    R = SO2(theta);
end

mat_skew_sym = [0 -1; 1 0];

adj_out = eye(3, class([t(:); theta(:)]));
adj_out(1:2, 1:2) = R;
adj_out(1:2, 3) = -mat_skew_sym * t(:);

end

