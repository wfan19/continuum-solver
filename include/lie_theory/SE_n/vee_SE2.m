function g_out = vee_SE2(SE2_in)
g_out = zeros(3, 1);

g_out(1:2) = SE2_in(1:2, 3);
g_out(3) = atan2(SE2_in(2, 1), SE2_in(1, 1));

end

