stickModel = StickModel();

%% Problem 1
disp("=== Problem 1 ===");
syms v1 v2 d2 d3 v4 v5 v6 d6;
dhTable = sym([]);
dhTable(1, :) = [0, 0, 0, 0]; % T01
dhTable(2, :) = [v1, 0, 0, -pi/2]; % T12
dhTable(3, :) = [v2, d2, 0, pi/2]; % T23
dhTable(4, :) = [-pi/2, d3, 0, 0]; % T34
dhTable(5, :) = [v4, 0, 0, -pi/2]; % T45
dhTable(6, :) = [v5 - pi/2, 0, 0, pi/2]; % T56
dhTable(7, :) = [v6 + pi/2, d6, 0, 0]; % T67

% Problem 1.a
disp("--- Problem 1.a ---");
fprintf("DH Parameters:\n");
disp(dhTable);

% Problem 1.b
disp("--- Problem 1.b ---");
% calcKinematics will print all frames and transforms automatically
[frames, transforms, jacobian] = calcKinematics([v1, v2, d3, v4, v5, v6], dhTable);
fprintf("FK =\n");
disp(frames{end});

% there's probably a way to do this substitution more concisely with
% cellfun(), but this works for now
v1 = 0; v2 = 0; d2 = 100; d3 = 100; v4 = 0; v5 = 0; v6 = 0; d6 = 100;
[F1, F2, F3, F4, F5, F6, F7] = frames{:};
stickModel.visualize([0; 0; 0], subs(F1), subs(F2), subs(F3), subs(F4), subs(F5), subs(F6), subs(F7));

%pause;
clearvars -except stickModel;

%% Problem 2
disp("=== Problem 2 ===");
% Problem 2.c
disp("--- Problem 2.c ---");
coefs = Traj_Planner.cubic_traj(0, pi/2, 0, 0, 0, 5);
fprintf("Trajectory Polynomial: theta2(t) = %f + %ft + %ft^2 + %ft^3\n", coefs(:));
figure;
fplot(Traj_Planner.traj_func(coefs), [0, 5]);
xlabel("Time [s]");
ylabel("Angle (theta2) [radians]");
title("Problem 2.c");

%pause;
clearvars -except stickModel;

%% Problem 3
disp("=== Problem 3 ===");
syms L1 theta1 theta2 d3;
dhTable = sym([]);
dhTable(1, :) = [0, 0, 0, 0]; % T01
dhTable(2, :) = [theta1, L1, 0, pi/2]; % T12
dhTable(3, :) = [pi/2 + theta2, 0, 0, pi/2]; % T23
dhTable(4, :) = [0, d3, 0, 0]; % T34

% Problem 3.a
fprintf("DH Parameters:\n");
disp(dhTable);

% Problem 3.b
disp("--- Problem 3.b ---");
[frames, transforms, jacobian] = calcKinematics([theta1, theta2, d3], dhTable);
fprintf("FK =\n");
disp(frames{end});

% Problem 3.e
disp("--- Problem 3.e ---");
syms px py pz;
p2 = [px; py; pz; 1] - frames{2} * [0; 0; 0; 1];
[az, el, r] = cart2sph(p2(1), p2(2), p2(3));
fprintf("theta1 = ");
disp(az);
fprintf("theta2 = ");
disp(el);
fprintf("d3 = ");
disp(r);

% Problem 3.f
disp("--- Problem 3.f ---");
fprintf("Jp(q) =\n");
disp(jacobian(1:3, :));

% Problem 3.g
disp("--- Problem 3.g ---");
L1 = 0.5; theta1 = 0; theta2 = 0; d3 = 0;
fprintf("V =\n");
V = jacobian * [0; 0.25; 0.02];
disp(V);
fprintf(" =\n");
disp(subs(V));

L1 = 50;
[F1, F2, F3, F4] = frames{:};
stickModel.visualize(subs(V(1:3)), subs(F1), subs(F2), subs(F3), subs(F4));

%pause;
clearvars -except stickModel;

%% Problem 4
disp("=== Problem 4 ===");
syms theta1 theta2 d3 L1 L2;
dhTable = sym([]);
dhTable(1, :) = [0, 0, 0, 0];
dhTable(2, :) = [theta1 + pi/2, L1, 0, pi/2];
dhTable(3, :) = [theta2, L2, 0, pi/2];
dhTable(4, :) = [0, d3, 0, 0];

% Problem 4.a
disp("--- Problem 4.a ---");
fprintf("DH Parameters:\n");
disp(dhTable);

% Problem 4.b
disp("--- Problem 4.b ---");
[frames, transforms, jacobian] = calcKinematics([theta1, theta2, d3], dhTable);
fprintf("FK =\n");
disp(frames{end});

% Problem 4.c
disp("--- Problem 4.c ---");
fprintf("J(q) =\n");
disp(jacobian);

% Problem 4.d
disp("--- Problem 4.d ---");
theta1 = 0; theta2 = 0; d3 = 0; L1 = 0.5; L2 = 0.5;
fprintf("V =\n");
V = jacobian * [0.1; 0.25; 0.02];
disp(V);
fprintf(" =\n");
disp(subs(V));

L1 = 50; L2 = 50;
[F1, F2, F3, F4] = frames{:};
stickModel.visualize(subs(V(1:3)), subs(F1), subs(F2), subs(F3), subs(F4));

%pause;
clearvars -except stickModel;

%%
%pause;
%stickModel.close();
%close all;
%clear stickModel;