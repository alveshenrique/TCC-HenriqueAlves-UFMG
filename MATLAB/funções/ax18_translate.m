% Função que translada a garra do robô. Suas entradas são:
% ax18: objeto da classe ax18.
% theta0: configuração das juntas no início do movimento.
% pd: quatérnio de translação correspondente à translação desejada.
% precison: precisão desejada para o movimento.
% Suas saídas são:
% theta: configurações das juntas após o movimento.
% p = quatérnion de translação correspondente à posição final.

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