function mat_out = mat_skew_sym(v)
% Create a 3D skew symmetric matrix with a vector v

mat_out = zeros(3, 3, class(v));
mat_out(2, 1) = v(3);
mat_out(3, 1) = -v(2);
mat_out(3, 2) = v(1);

mat_out = mat_out + -mat_out';
end

