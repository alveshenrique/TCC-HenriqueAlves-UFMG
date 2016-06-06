clc;
close all;

%% Portas de comunicação para o braço e a base móvel
arm_port = {'COM9'};
base_port = 7;

%% Manipulator (physical) object
Ax18 = AX18(arm_port);
Ax18.start

%% D&H Parameters Vectors
DH_Alpha   =  [-pi/2     0       -pi/2      0       -pi/2    0 ];  % Alpha
DH_A          =  [  0    0.159    0      0.02225     0      0 ];  % A
DH_Theta   =  [  0        0      -pi/2      0       -pi/2     0 ];  % Theta
DH_D         =  [ 0.167   0        0     0.0815   0.041    0 ];  % D
DH_virtual =  [0,         0,       0,        1,         0,      0]; % Virtual joint definition

DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model
AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

%% Initial Position for the Robot Arm
theta0 = [0    1.97*-pi/3   1.81*pi/3    0    0]';
thetab = theta0;
x_arm = AX18DQ.fkm(thetab);

%% Esta parte do código permite colocar ao robô manipulador na pose inicial
for i=1:5
    Ax18.joint(i,theta0(i),0.05);
end