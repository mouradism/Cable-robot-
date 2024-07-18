classdef TestControle < matlab.unittest.TestCase
    properties
        Args
        q
        dq
        x
        t
    end

    methods(TestMethodSetup)
        function setupArgs(testCase)
            testCase.Args.t_ = 0;
            testCase.Args.E_ = 0;
            testCase.Args.TT = [];
            testCase.Args.FF = [];
            testCase.Args.g = 9.8;
            testCase.Args.m1 = 6;
            testCase.Args.m2 = 4;
            testCase.Args.m3 = 1;
            testCase.Args.l1 = 0.5;
            testCase.Args.l2 = 0.5;
            testCase.Args.l3 = 0.3;
            testCase.Args.La = 2.0;
            testCase.Args.Lb = 2.0;
            testCase.Args.r = 0.25;
            testCase.Args.Xi = [-0.5; -0.4];
            testCase.Args.alpha = -0.0 * pi;

            testCase.Args.XL = [testCase.Args.Lb / 2, -testCase.Args.Lb / 2, testCase.Args.Lb / 2, -testCase.Args.Lb / 2; 0, 0, 0, 0];
            testCase.Args.Xf = testCase.Args.Xi + [0.40; -0.40];
            testCase.Args.J1 = (testCase.Args.m1 * testCase.Args.l1^2) / 3;
            testCase.Args.J2 = (testCase.Args.m2 * testCase.Args.l2^2) / 3;
            testCase.Args.J3 = (testCase.Args.m3 * testCase.Args.l3^2) / 3;

            % Inverse Kinematics
            target = testCase.Args.Xi + testCase.Args.l3 * [cos(testCase.Args.alpha); sin(testCase.Args.alpha)];
            testCase.q = invKIN(target, testCase.Args.Xi, testCase.Args);

            % Initial conditions
            testCase.dq = [0; 0; 0];
            testCase.Args.ivp = [testCase.q; testCase.dq];
            testCase.x = testCase.Args.ivp;
            testCase.t = 0;
            testCase.Args.Xf = testCase.Args.Xi + [0.40; -0.40];

            testCase.Args.lj = [1, 1, 1; 0.5, 0, 0; 1, 1, 0; 1, 0.5, 0] * [testCase.Args.l1; testCase.Args.l2; testCase.Args.l3];
            testCase.Args.XL = [+testCase.Args.Lb / 2, -testCase.Args.Lb / 2, +testCase.Args.Lb / 2, -testCase.Args.Lb / 2; -0 * testCase.Args.La, 0, 0, -0 * testCase.Args.La];

            if isempty(testCase.Args.lj) || isempty(testCase.Args.XL)
                disp('Invalid configuration');
            else
                target = testCase.Args.Xi + testCase.Args.l3 * [cos(testCase.Args.alpha); sin(testCase.Args.alpha)];
                testCase.q = invKIN(target, testCase.Args.Xi, testCase.Args);
            end

            testCase.dq = [0; 0; 0];
        end
    end

    methods(Test)
        function testControleFunction(testCase)
            [Tau, F] = controle(testCase.t, testCase.x, testCase.Args);

            % Check the shapes of the outputs
            testCase.verifyEqual(size(Tau), [4, 1]);
            testCase.verifyEqual(F, 0);

            % Check some basic values (these will depend on the mock functions)
            expected_Tau = [0.1; 0.1; 0.1; 0.1];
            testCase.verifyEqual(Tau, expected_Tau, 'AbsTol', 1e-3);
        end
    end
end
%Run the tests in MATLAB using the command runtests('TestControle').
