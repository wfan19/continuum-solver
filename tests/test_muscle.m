function tests = test_muscle
    tests = functiontests(localfunctions);
end

%% Fixtures
% Initialize both of our muscles with a standard length
function setup(testCase)
    l_0 = 1;
    testCase.TestData.l_0 = l_0;

    testCase.TestData.muscle2d = Muscle(SE2, l_0);
    testCase.TestData.muscle3d = Muscle(SE3, l_0);

    g_0_2d = SE2.hat([-1; 2; -3]);
    g_0_3d = SE3.hat(eul2rotm([0, -pi, 0], "xyz"), [-1; -2; -3]);

    testCase.TestData.muscle2d.g_0 = g_0_2d;
    testCase.TestData.muscle3d.g_0 = g_0_3d;
end

% Delete the arms
function teardown(testCase)
    delete(testCase.TestData.muscle2d);
    delete(testCase.TestData.muscle3d);
end

%% Set and access functions
% Set individual muscle length/shear/curvature properties, and then
% validate the g_circ_right value
function test_set_properties_2d(testCase)
    muscle = testCase.TestData.muscle2d;
    verifyEqual(testCase, muscle.l, testCase.TestData.l_0);

    muscle.l = 2;
    muscle.true_shear = 0.5;
    muscle.true_curvature = -1;

    g_circ_right_correct = [1; 0.5; -1] * 2;
    verifyEqual(testCase, muscle.g_circ_right, g_circ_right_correct);
end

% Set individual muscle length/shear/curvature properties, and then
% validate the g_circ_right value
function test_set_properties_3d(testCase)
    muscle = testCase.TestData.muscle3d;
    verifyEqual(testCase, muscle.l, testCase.TestData.l_0);
    verifyEqual(testCase, muscle.true_curvature, [0; 0; 0]);

    muscle.l = 2;
    muscle.true_shear = [0.1; -0.1];
    muscle.true_curvature = [1; -1; 1];
    g_circ_right_correct = 2 * [1; 0.1; -0.1; 1; -1; 1];
    verifyEqual(testCase, muscle.g_circ_right, g_circ_right_correct);
end

% Set g_circ_right value, and then validate the length/shear/curvature
% properties
function test_access_properties_2d(testCase)
    muscle = testCase.TestData.muscle2d;
    
    g_circ_right = 2 * [1; 0.5; -1];
    muscle.g_circ_right = g_circ_right;

    verifyEqual(testCase, muscle.true_curvature, -1);
    verifyEqual(testCase, muscle.true_shear, 0.5);
end

% Set g_circ_right value, and then validate the length/shear/curvature
% properties
function test_access_properties_3d(testCase)
    muscle = testCase.TestData.muscle3d;
    g_circ_right = 2 * [1; 0.1; -0.1; 1; -1; 1];
    muscle.g_circ_right = g_circ_right;

    verifyEqual(testCase, muscle.true_curvature, [1; -1; 1]);
    verifyEqual(testCase, muscle.true_shear, [0.1; -0.1]);
end

% Test the copy constructor
function test_copy(testCase)
    muscle2d_2 = copy(testCase.TestData.muscle2d);
    muscle2d_2.l = 2;

    verifyEqual(testCase, testCase.TestData.muscle2d.l, 1);
    verifyEqual(testCase, muscle2d_2.l, 2);
end

%% Test exponential map
% 
function test_expm_2d(testCase)
    muscle = testCase.TestData.muscle2d;

    g_circ_right = [1; 2; 3];
    t = linspace(0, 1, 20);

    muscle.g_circ_right = g_circ_right;
    poses = muscle.calc_posns(t=t);
    
    mat_poses_truth = zeros(3, 20);
    for i = 1 : length(t)
        mat_poses_truth(:, i) = SE2.vee(muscle.g_0 * se2.expm(g_circ_right * t(i)));
    end

    verifyEqual(testCase, poses, mat_poses_truth);
end

function test_expm_3d(testCase)
    muscle = testCase.TestData.muscle3d;

    g_circ_right = [1; 2; 3; 4; 5; 6];
    t = linspace(0, 1, 20);

    muscle.g_circ_right = g_circ_right;
    poses = muscle.calc_posns(t=t);
    
    mat_poses_truth = zeros(6, 20);
    for i = 1 : length(t)
        mat_poses_truth(:, i) = SE3.vee(muscle.g_0 * se3.expm(g_circ_right * t(i)));
    end

    verifyEqual(testCase, poses, mat_poses_truth);
end
