function rotation = rpy_rotation(alpha, beta, gamma)
% 固定角坐标系

    rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha), cos(alpha)];

    ry = [cos(beta), 0, sin(beta);
          0, 1, 0;
          -sin(beta), 0, cos(beta)];

    rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma), cos(gamma), 0;
          0, 0, 1];

    rotation = rz * ry * rx;
end
