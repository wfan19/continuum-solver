% Construct the matrix representation of an SE2 pose group element
function SE2_out = SE2(x, y, theta)
    if nargin == 1
        h = x;
        x = h(1);
        y = h(2);
        theta = h(3);
    end
    
    SE2_out = eye(3, class([x, y, theta]));
    SE2_out(1:2, 1:2) = SO2(theta);
    SE2_out(1:2, 3) = [x; y];
end
