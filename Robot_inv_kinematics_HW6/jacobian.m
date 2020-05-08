% syms x y z
% jacobian([x*y*z, y^2, x + z], [x, y, z])
syms phi1 phi2 phi3
Ts0_1 = [cos(phi1), -1*sin(phi1), 0; sin(phi1), cos(phi1), 12.5; 0, 0, 1];
Ts1_2 = [cos(phi2), -1*sin(phi2), 0; sin(phi2), cos(phi2), 12; 0, 0, 1];
Ts2_3 = [cos(phi3), -1*sin(phi3), 0; sin(phi3), cos(phi3), 12; 0, 0, 1];
Ts3_4 = [cos(0), -1*sin(0), 0; sin(0), cos(0), 12; 0, 0, 1];

x_g = 35;
y_g = 21;
e = [0,48.5];
T0_5 = Ts0_1*Ts1_2*Ts2_3*Ts3_4*[0;0;1];

J = jacobian([T0_5(1), T0_5(2)], [phi1, phi2, phi3]);
J_n = subs(J, [phi1, phi2, phi3], [0, 0, 0]);
% J_inv = pinv(J_n);
% d_phi = J_inv*[0.5;0.6];
% rad2deg(d_phi(1))

C = computeCost(e(1),e(2),x_g,y_g);
fprintf('Cost: %f\n',C);
lmbda = -0.1;
p = [0;0;0];

while C > 0.5
    [dx,dy] = gradientDesc(e(1),e(2),x_g,y_g);
    de = lmbda*[dx;dy];
    J_n = subs(J, [phi1, phi2, phi3], [deg2rad(p(1)), deg2rad(p(2)), deg2rad(p(3))]);
    fprintf('Finding inv..\n');
    J_inv = pinv(J_n);
    fprintf('Found inv\n');
    d_phi = J_inv*de;
    p = p+rad2deg(d_phi);
    disp(p);
    e = e+de;
    C = computeCost(e(1),e(2),x_g,y_g);
%     fprintf('Cost: %f\n',C);
    disp(C)
%     fprintf('------------------------------\n');
end

% subs(T0_5, [phi1, phi2, phi3], [0, 45, 90])
function [dx,dy] = gradientDesc(x,y,x_g,y_g)
    d_g = computeCost(x,y,x_g,y_g);
    dx = (x-x_g)/d_g;
    dy = (y-y_g)/d_g;    
end

function c = computeCost(x,y,x_g,y_g)
    c = sqrt((x-x_g)^2+(y-y_g)^2);
end