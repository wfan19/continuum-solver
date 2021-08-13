
function se2_out = se2(vx, vy, omega)
    mat_skew_sym = [0, -omega; omega, 0];
    se2_out = zeros(3, class([vx, vy, omega]));
    se2_out(1:2, 1:2) = mat_skew_sym;
    se2_out(1:2, 3) = [vx; vy];
end