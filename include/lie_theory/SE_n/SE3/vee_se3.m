function v_twist = vee_se3(mat_se3)
% Extract linear and angular velocities from an se3 twist matrix

v_vel = mat_se3(1:3, 4);

mat_omega = mat_se3(1:3, 1:3);
v_omega = [mat_omega(3, 2);
            -mat_omega(3, 1);
            mat_omega(2, 1)
];

v_twist = [v_vel; v_omega];

end

