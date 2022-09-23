classdef Traj_Planner
    %% Trajectory Planner
    % generates trajectories based on boundary conditions
    methods(Static)
        function coefs = cubic_traj(q0, q1, v0, v1, t0, t1)
            %% cubic_traj
            % calculates the coefficients of a cubic polynomial that satisfies
            % the given boundary conditions.
            %
            % this function generalizes to higher dimensions. parameters
            % q0, q1, v0, and v1 are row vectors, with as many elements as
            % there are dimensions.
            %
            %   q0: position at t0, as a row vector
            %   q1: position at t1, as a row vector
            %   v0: velocity at t0, as a row vector
            %   v1: velocity at t1, as a row vector
            %   t0: initial value of t
            %   t1: final value of t
            arguments
                q0 (1, :)
                q1 (1, :)
                v0 (1, :)
                v1 (1, :)
                t0 (1, 1) {mustBeNumeric}
                t1 (1, 1) {mustBeNumeric}
            end
            C = [1  t0  t0^2    t0^3;
                 0   1  2*t0  3*t0^2;
                 1  t1  t1^2    t1^3;
                 0   1  2*t1  3*t1^2];
            assert(det(C) ~= 0, "Coefficient matrix is degenerate. Cannot calculate trajectory.");
            y = vertcat(q0, v0, q1, v1);
            coefs = inv(C) * y

        end

        function coefs = quintic_traj(q0, q1, v0, v1, a0, a1, t0, t1)
            arguments
                q0 (1, :)
                q1 (1, :)
                v0 (1, :)
                v1 (1, :)
                a0 (1, :)
                a1 (1, :)
                t0 (1, 1)
                t1 (1, 1)
            end
            C = [1  t0  t0^2    t0^3     t0^4     t0^5;
                 0   1  2*t0  3*t0^2   4*t0^3   5*t0^4;
                 0   0     2    6*t0  12*t0^2  20*t0^3;
                 1  t1  t1^2    t1^3     t1^4     t1^5;
                 0   1  2*t1  3*t1^2   4*t1^3   5*t1^4;
                 0   0     2    6*t1  12*t1^2  20*t1^3;];
            assert(det(C) ~= 0, "Coefficient matrix is degenerate. Cannot calculate trajectory.");
            y = vertcat(q0, v0, a0, q1, v1, a1);
            coefs = linsolve(C, y)
        end

        function p = traj_eval(coefs, t)
            dimension = length(coefs(1, :));
            p = zeros(dimension, 1);
            for idx = 1:dimension
                p(idx) = polyval(flip(coefs(:, idx)), t);
            end
        end

        function func = traj_func(coefs)
            func = @(t) Traj_Planner.traj_eval(coefs, t);
        end
    end
end