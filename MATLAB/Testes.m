% Exemplo AX18_kinematic_control comentado para melhor entendimento e
% futuros testes.

close all;
clear all;
clear classes;
clc;

%Create a new DQ_kinematics object with the AX18 arm standard Denavit-Hartenberg parameters           
ax18 = DQ_AX18;

%Initial configuration
theta=[0 0 0 0 0]'; 

% function set_effector(obj,effector)
%             % dq.set_effector(effector) sets the pose of the effector
%             obj.effector = DQ(effector);
%         end
% 
%       dq = DQ(v) %Where is a 8-, 4-, or 1-dimension vector with
%               the dual quaternions coefficients. The 4- and 1-dimension vectors are special cases,
%               where the 4-dimension vector means quaternion definition,
%               whilst the 1-dimension vector means a scalar, both in dual quaternion space.


%Configuring the end-effector:
ax18.set_effector(cos(pi/4)+DQ.i*sin(pi/4))

%  function q = fkm(obj,theta, ith)
%             %   dq = fkm(theta) calculates the forward kinematic model and
%             %   returns the dual quaternion corresponding to the
%             %   end-effector pose. This function takes into account the
%             %   displacement due to the base's and effector's poses.
%             %
%             %   theta is the vector of joint variables
%             %   
%             %   dq = fkm(theta, ith) calculates the FKM up to the ith link.
%
%p = translation(dq) returns the translation of the unit dual quaternion dq,
%assuming  dq=r + DQ.E * p * r * (0.5), that is, the translation followed
%by rotation movement.

p = translation(ax18.fkm(theta));
% pd = translation(ax18.fkm([pi/4 -pi/4 pi/2 0 pi/4]));

% pd = p-DQ.i*281.4988+DQ.j*281.4988;
pd = p-DQ.i*281.4988+DQ.j*281.4988;

error = pd - p;
while norm(vec4(error)) > 0.1
    x = ax18.fkm(theta);
    p = translation(x);
     
%     function Jp = jacobp(J,x)
%             %Given the dual quaternion Jacobian J and the corresponding
%             %dual quaternion, Jp = jacobp(J,x) returns the translation Jacobian; that it,
%             %the Jacobian that satisfies the relation dot_p = Jp * dot_theta,
%             %where dot_p is the time derivative of the
%             %translation quaternion and dot_theta is the time derivative of
%             %the joint vector.
%     function J = jacobian(obj,theta)
%             % J = jacobian(theta) returns the dual quaternion Jacobian, where
%             % theta is the vector of joint variables
    
    Jp = ax18.jacobp(ax18.jacobian(theta),x);    
    error = pd - p;
    theta = theta + pinv(Jp)*0.1*vec4(error);
    plot(ax18,theta);
    drawnow;
end
