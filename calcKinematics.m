function [frames, transforms, jacobian] = calcKinematics(jointVars, dhTable)
    numJoints = length(jointVars);
    [numParams, ~] = size(dhTable);
    assert(numParams == numJoints + 1);
    % row 1 of the DH table is always the transformation between the ground
    % frame and the first joint, and SHOULD NOT depend on any joint
    % variables.
    % frames{n} = frame n
    % transforms{n} = transformation from frame n -1 to frame n
    % frames{end} = end effector frame

    % frame 0 is implicitly an identity
    frames = cell(numJoints + 1, 1);
    transforms = cell(numJoints + 1, 1);
    jacobian = sym(zeros(6, numJoints));

    % first, calculate all of the joint frames (as well as the end effector
    % frame) and the transformation matrices between them.
    F = sym(eye(4)); % initialized to frame 0
    for frameIdx = 1:numJoints + 1
        % previous frame to current frame, per frameIdx
        T = TDHSym.tdh2(dhTable(frameIdx, :));

        F = F * T;
        % F is now the frame corresponding to frameIdx
        
        fprintf("T%d%d =\n", frameIdx - 1, frameIdx);
        disp(T);
        fprintf("F%d =\n", frameIdx);
        disp(F);

        frames{frameIdx} = F;
        transforms{frameIdx} = T;
    end

    % now, do the calculus to find the Jacobian.
    p = F(1:3, 4);
    for jointIdx = 1:numJoints
        jacobian(1:3, jointIdx) = diff(p, jointVars(jointIdx));
        % inspiration: during my time in FTC, I had to find the heading of the
        % robot, even as the IMU rotated along all 3 axes of rotation. i
        % used AHRS to find the gravity vector, then dotted that vector
        % with the angular velocity vector to get only the portion that was
        % around the gravity vector. here we need to find the rotation
        % around all 3 axes, which is just like dotting the i, j, and k
        % unit vectors with the angular velocity. or in this case, we can
        % apply the chain rule to get the jacobian.
        % Jo = rotation axis * d[rotation amount]/d[joint variable]
        % for DH parameters, the rotation axis is the z-axis of the joint
        % frame, and the derivative can be derived from the DH table. since
        % prismatic joints have a constant value for the DH parameter
        % theta, this code handles prismatic joints as well as revolute
        % joints.
        f = frames{jointIdx};
        jacobian(4:6, jointIdx) = f(1:3, 3) * diff(dhTable(jointIdx + 1, 1), jointVars(jointIdx));
    end

    fprintf("rank J = %.0f\n", rank(jacobian));
end