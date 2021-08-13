% Construct the matrix representation of an SO2 orientation group eI just lement
function SO2_out = SO2(theta)
     SO2_out = [cos(theta), -sin(theta); sin(theta), cos(theta)];
     if class(theta) ~= "sym" % Round if we are constructing a numeric matrix
        SO2_out = round(SO2_out, 8); % 8 places should be sufficient?
     end
end