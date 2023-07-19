function tests = test_muscle
    tests = functiontests(localfunctions);
end

function setup(testCase)
    l_0 = 1;
    testCase.TestData.l_0 = l_0;

    testCase.TestData.muscle2d = Muscle(SE2, l_0);
    testCase.TestData.muscle3d = Muscle(SE3, l_0);
end

function teardown(testCase)
    delete(testCase.TestData.muscle2d);
end

function test_set_properties_2d(testCase)
    muscle = testCase.TestData.muscle2d;
    verifyEqual(testCase, muscle.l, testCase.TestData.l_0);

    muscle.l = 2;
    muscle.true_shear = 0.5;
    muscle.true_curvature = -1;

    g_circ_right_correct = [1; 0.5; -1] * 2;
    verifyEqual(testCase, muscle.g_circ_right, g_circ_right_correct);
end

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

function test_copy(testCase)
    muscle2d_2 = copy(testCase.TestData.muscle2d);
    muscle2d_2.l = 2;

    verifyEqual(testCase, testCase.TestData.muscle2d.l, 1);
    verifyEqual(testCase, muscle2d_2.l, 2);
end
