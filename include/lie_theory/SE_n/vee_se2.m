function h_out = vee_se2(se2_in)

h_out = zeros(3, 1);

h_out(1:2) = se2_in(1:2, 3);
h_out(3) = se2_in(2, 1);

end

