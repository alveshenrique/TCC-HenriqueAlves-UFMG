% Fun��o que translada a garra do rob�. Suas entradas s�o:
% ax18: objeto da classe ax18.
% theta0: configura��o das juntas no in�cio do movimento.
% pd: quat�rnio de transla��o correspondente � transla��o desejada.
% precison: precis�o desejada para o movimento.
% Suas sa�das s�o:
% theta: configura��es das juntas ap�s o movimento.
% p = quat�rnion de transla��o correspondente � posi��o final.

function [theta,flag] = ax18_translate(ax18,theta0,pd,precision)
theta = theta0;

x = ax18.fkm(theta);
p = translation(x);
error = pd - p;
k=0;
while (norm(vec4(error)) > precision)&&(k<50)
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
    k=k+1;
end

flag =1;
end