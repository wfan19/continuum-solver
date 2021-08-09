% Construct the matrix representation of an SE2 pose group element
function SE2_out = SE2(x, y, theta)
    SE2_out = eye(3, class([x, y, theta]));
    SE2_out(1:2, 1:2) = SO2(theta);
    SE2_out(1:2, 3) = [x; y];
end
