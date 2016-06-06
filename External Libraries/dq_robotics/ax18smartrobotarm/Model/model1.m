%% D&H model for AX-18 Smart Robot Arm


% July 2012
% Ernesto Lana
% Robotics toolbox by Peter Corke (v9.5)
% DQ toolbox by Bruno Adorno

clear all;
close all;
clear classes;
clc;

%% Model Definitions
% ----- Comau Manipulator Model Definition -----

%L(i)= LINK([theta   D          A       alpha   sigma offset], CONVENTION)

L(1) = Link([ 0     85.750     79.500   -pi/2    0      0    ], 'standard');  % Joint 1
L(2) = Link([ 0      0.000    159.000    0       0      0    ], 'standard');  % Joint 2
L(3) = Link([ 0      0.000      0.000    0       0      0    ], 'standard');  % Joint 3
L(4) = Link([ -pi/2  0.000      0.000   -pi/2    0      -pi/2], 'standard');  % Virtual Link
L(5) = Link([ 0     81.500     22.250    0       0      0    ], 'standard');  % Joint 4
L(6) = Link([ -pi/2 41.000      0.000   -pi/2    0      -pi/2], 'standard');  % Virtual Link
L(7) = Link([ 0      0.000      0.000    0       0      0    ], 'standard');  % Joint 5

AX18RA = SerialLink(L);  % Defines the robot model

% ---- Robot Arm Model w/Dual Quaternions ----

% D&H Parameters Vectors
DH_Alpha   = [-pi/2  0 0 -pi/2 0 -pi/2 0];  % Alpha
DH_A       = [79.5 159  0 0 22.25 0 0];  % A
DH_Theta   = [0 0 0 -pi/2 0 -pi/2 0];  % Theta
DH_D       = [85.75 0 0 0 81.5 41 0];  % D
DH_virtual =    [0,0,0,1,0,1,0]; % Virtual joint definition

DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model

AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

%% Initial Parameters and Plots

ThetaI = [0 0 0 0 0 0 0]; % Initial position for the manipulator
ThetaD = [0 0 -pi/2 0 pi/2 0]; % Desired final position for the manipulator

% Manipulator model plot
figure(1)
plot(AX18DQ,ThetaI)
figure(2);
AX18RA.plot(ThetaI,'noshadow','nobase','noname')

