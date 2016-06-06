%% D&H control test #3 for AX-18 Smart Robot Arm

% August 2012
% Ernesto Lana
% Robotics toolbox by Peter Corke (v9.5)
% DQ toolbox by Bruno Adorno
% AX18 Robot Arm functions by Ernesto Lana

clear all;
close all;
clear classes;
clc;

%% AX18 manipulator model definition

%L(i)= LINK([theta   D          A       alpha   sigma offset], CONVENTION)

L(1) = Link([ 0    167.000      0.000   -pi/2    0      -pi/2], 'standard');  % Joint 1
L(2) = Link([ 0      0.000    159.000    0       0      0    ], 'standard');  % Joint 2
L(3) = Link([ -pi/2  0.000      0.000   -pi/2    0      -pi/2], 'standard');  % Joint 3
L(4) = Link([ 0     81.500     22.250    0       0      0    ], 'standard');  % Virtual Link
L(5) = Link([ -pi/2 41.000      0.000   -pi/2    0      -pi/2], 'standard');  % Joint 4
L(6) = Link([ 0      0.000      0.000    0       0      0    ], 'standard');  % Joint 5

AX18RA = SerialLink(L);  % Defines the robot model


% ---- Robot arm model w/dual quaternions ----

% D&H Parameters Vectors
DH_Alpha   = [-pi/2  0 -pi/2 0 -pi/2 0];  % Alpha
DH_A       = [0 159  0 22.25 0 0];  % A
DH_Theta   = [0 -pi/2 -pi/2 0 -pi/2 0];  % Theta
DH_D       = [167 0 0 81.5 41 0];  % D
DH_virtual =    [0,0,0,1,0,0]; % Virtual joint definition

DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model

lef = 170;  % Length of end effector (from last link)
eff = 1 + DQ.E*(-0.5*lef*DQ.j);  % End effector definition

AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

AX18DQ.set_effector(eff)

%% Robot communication ====
% ax=AX18; % Robot object
% ax.start % Starts communication with the robot
% =============================

%% Robot control initial parameters

eps = 1; % Acceptable Error to Stop Control Loop
Gain = 0.1;  % Controller Gain
Error = eps+1; % Initial Error
vel = 4;  % Joint velocity (percentage)

% Sequence 1

Joint=[0 -40 90 0 0]'; %Initial position for the manipulator
Joint = Joint*pi/180;


% Movement Sequence

% for i=1:5
%     ax.joint(i,Joint(i)*180/pi,10)
% end

Jointd=[0 60 80 0 0]';  % Desired position for the manipulator
Jointd=Jointd*pi/180;
Jointd1 = Jointd;
xd = AX18DQ.fkm(Jointd);

pause()

k = 1;
while norm(Error) > eps
    JacobR = AX18DQ.jacobian(Joint);
    Error = vec8(AX18DQ.fkm(Jointd)-AX18DQ.fkm(Joint));
    Joint = Joint + pinv(JacobR)*0.05*Error;
    % Updates position in the manipulator
    Joint1 = (Joint*180)/pi;
    Theta(k,:) = Joint1;
    k = k+1;
%     for i=1:5
%         ax.joint(i,Joint1(i),vel)
%     end
end

Jointd=[0 -40 90 0 0]'; %Initial Position for the Robot Arm
Jointd = Jointd*pi/180;
Jointd2 = Jointd;
xd2 = AX18DQ.fkm(Jointd);
Error = eps+1;
pause()

while norm(Error) > eps
    JacobR = AX18DQ.jacobian(Joint);
    Error = vec8(AX18DQ.fkm(Jointd)-AX18DQ.fkm(Joint));
    %Joint = Joint + dinv(JacobR,0.01)*0.1*Error;
    Joint = Joint + pinv(JacobR)*0.05*Error;
    % Updates position
    Joint1 = (Joint*180)/pi;
%     for i=1:5
%         ax.joint(i,Joint1(i),vel)
%     end
end