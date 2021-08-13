function SE3_out = SE3(R, t)

if class(R) ~= "sym"
    R = round(R, 5);
end

SE3_out = eye(4, class([R, t(:)]));

SE3_out(1:3, 1:3) = R;
SE3_out(1:3, 4) = t;

end

