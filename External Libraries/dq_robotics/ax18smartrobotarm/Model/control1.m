%% D&H control test #1 for AX-18 Smart Robot Arm

% July 2012
% Ernesto Lana
% Robotics toolbox by Peter Corke (v9.5)
% DQ toolbox by Bruno Adorno

clear all;
close all;
clear classes;
clc;

%% Model Definitions
% ----- Manipulator Model Definition -----

%L(i)= LINK([theta   D          A       alpha   sigma offset], CONVENTION)

L(1) = Link([ 0    167.000      0.000   -pi/2    0      0    ], 'standard');  % Joint 1
L(2) = Link([ 0      0.000    159.000    0       0      -pi/2], 'standard');  % Joint 2
L(3) = Link([ -pi/2  0.000      0.000   -pi/2    0      -pi/2], 'standard');  % Joint 3
L(4) = Link([ 0     81.500     22.250    0       0      0    ], 'standard');  % Virtual Link
L(5) = Link([ -pi/2 41.000      0.000   -pi/2    0      -pi/2], 'standard');  % Joint 4
L(6) = Link([ 0      0.000      0.000    0       0      0    ], 'standard');  % Joint 5

AX18RA = SerialLink(L);  % Defines the robot model

% ---- Robot Arm Model w/Dual Quaternions ----

% D&H Parameters Vectors
DH_Alpha   = [-pi/2  0 -pi/2 0 -pi/2 0];  % Alpha
DH_A       = [0 159  0 22.25 0 0];  % A
DH_Theta   = [0 0 -pi/2 0 -pi/2 0];  % Theta
DH_D       = [167 0 0 81.5 41 0];  % D
DH_virtual =    [0,0,0,1,0,0]; % Virtual joint definition

DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model

AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

%% Initial Parameters and Plots

Theta = [0 0 0 0 0 0]; % Initial joint position for the manipulator
ThetaD = [0 0 0 0 0 pi/2]; % Desired joint position for the manipulator

% Manipulator model plot
% figure(1)
% plot(AX18DQ,ThetaI)
figure;
AX18RA.plot(Theta,'noshadow','nobase','noname')

%% Robot Control

Gain = 0.2;  % Controller Gain
Error = eps+1; % Initial Error
Xd = AX18DQ.fkm(ThetaD);  % Desired position
JointV = [0 0 0 0 0 0];  % Initial joint 


% Sequence 1
while norm(Error) > eps
    Jacob = ArmDQModel.jacobian(Theta);  % Arm joint positions Jacobian
    Xm = AX18DQ.fkm(Theta); % Current measured position
    Error = Xd.q-Xm.q; % Error Calculation
    Theta = Theta + pinv(Jacob)*Gain*Error; % Controller algorithm
    Xh_pos = ArmDQModel.fkm(Theta); % Updated Position
    X_error = pinv(Xh_ant) .*  Xh_pos;  % Displacement of the Arm
    Xr_ant = getDualPosition(comau)*(cos(pi/2)+DQ.i*sin(pi/2)); % Current Robot Arm Position
    JacobR = RobotDQ.jacobian(Theta);
     X_error = (1+DQ.E*translation(X_error))*P(X_error);
    Xr_pos = Xr_ant .* (X_ra * X_error); % Robot Position Updating
    Xr_dis = translation(Xr_pos) - translation(Xr_ant);
    comau.setJointPositions(Theta*180/pi); % Sends Update Position Command to the Robot Arm
    Theta=comau.getJointPositions()*(pi/180); % Robot Arm Joints Current Position
    % Graphical Position Updating
    subplot (1,2,1)
    %plot(ArmModel, Theta'); % Arm
    subplot (1,2,2)
    %plot(RobotArm, [Joint;0]');  % Robot Arm
end

pause()
% Sequence 2

Xd = ArmDQModel.fkm(Theta0);  % Final (Desired) Position

Error = eps+1; % Initial Error


while norm(Error) > eps
    Jacob = ArmDQModel.jacobian(Theta);  % Arm Joint Positions Jacobian
    Xm = ArmDQModel.fkm(Theta); % Current Position
    Xh_ant = Xm;
    Error = Xd.q-Xm.q; % Error Calculation
    Theta = Theta + pinv(Jacob)*Gain*Error; % Controller Algorithm
    Xh_pos = ArmDQModel.fkm(Theta); % Updated Position
    X_error = pinv(Xh_ant) .*  Xh_pos;  % Displacement of the Arm
    Xr_ant = getDualPosition(comau)*(cos(pi/2)+DQ.i*sin(pi/2)); % Current Robot Arm Position
    JacobR = RobotDQ.jacobian(Theta);
     X_error = (1+DQ.E*translation(X_error))*P(X_error);
    Xr_pos = Xr_ant .* (X_ra * X_error); % Robot Position Updating
    Xr_dis = translation(Xr_pos) - translation(Xr_ant);
    comau.setJointPositions(Theta*180/pi); % Sends Update Position Command to the Robot Arm
    Theta=comau.getJointPositions()*(pi/180); % Robot Arm Joints Current Position
    % Graphical Position Updating
    subplot (1,2,1)
    %plot(ArmModel, Theta'); % Arm
    subplot (1,2,2)
    %plot(RobotArm, [Joint;0]');  % Robot Arm
end


