%% D&H control test #1 for AX-18 Smart Robot Arm

% August 2012
% Ernesto Lana
% Robotics toolbox by Peter Corke (v9.5)
% DQ toolbox by Bruno Adorno
% AX18 Robot Arm functions by Ernesto Lana

clear all;
close all;
clear classes;
clc;

%% Model definitions

% =================== Human arm model ===========================
Lse=350;  % Shoulder - Elbow
Leh=300;   % Elbow - Hand

% Links Definition
% D&H Parameters:  Alpha   A  Theta  D  Sigma
        L(1) = Link([ 0     Lse   0    0    0], 'standard');  % Link 1
        L(2) = Link([ 0     Leh   0    0    0], 'standard');  % Link 2
        L(3) = Link([-pi/2  0   -pi/2  0    0], 'standard');  % Link 3
        L(4) = Link([ 0     0     0    0    0], 'standard');  % Link 4

ArmModel=SerialLink(L);  % Defines the Arm Model as a 4-Link Robot

% D&H Parameters Vectors
DH_Alpha = [0 0 -pi/2 0];  % Alpha
DH_A     = [Lse Leh 0 0];  % A
DH_Theta = [0 0 -pi/2 0];  % Theta
DH_D     = [0 0   0   0];  % D
DH_virtual = [0 0 0 0]; % Virtual joint definition

DH_ArmMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual]; % D&H Parameters Matrix for the Arm Model

ArmDQ = DQ_kinematics(DH_ArmMatrix,'standard'); % Defines Arm Model Using Dual Quaternions


% ================ AX18 manipulator model definition ==============

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

AX18DQ.effector=eff;  % Sets effector

%% Robot communication ====
ax=AX18; % Robot object
ax.start % Starts communication with the robot
% =============================

%% Robot control


Theta0 =[-pi/2  pi/2 -pi/2 0]';  %Initial Position
Theta = Theta0;  % Current Position for the Arm
Joint=[0 0 pi/2 0 0]'; %Initial Position for the Robot Arm

for i=1:5
    ax.joint(i,Joint(i)*180/pi,10)
end
% figure;
% Arm Model Plot
% subplot(1,2,1)
% title ('Human Arm Model')
% PlotOp={'nobase','noname','noshadow','ortho'};
% %plot(ArmModel, Theta',PlotOp{:});
%ArmModel.plot(Theta','noshadow','nobase','noname')
% plot(ArmDQ,Theta')
% axis equal;
% axis([-200,600,-600,200,-1000,1000]);
% grid off;
% view(-180,90)
% hold on
% % Robot Arm Plot
% subplot(1,2,2)
% % title ('Robot Arm Model')
% % PlotOp={'nobase','noname','noshadow','ortho'};
% jpos = ax.getjoints;
% plot(AX18DQ,jpos*pi/180)
% % plot(RobotArm,[jpos(1:3);0;jpos(4:5)],PlotOp{:}); 
% axis equal;
% axis([-200,300,-200,200,-50,400]);
%return
% grid off;
% view(-180,0)
% hold on;
pause()
eps = 0.1; % Acceptable Error to Stop Control Loop
Theta1 = [-3*pi/8 3*pi/4 -pi/2 0]'; % Final Arm Position

Xd = ArmDQ.fkm(Theta1);  % Final (Desired) Position

Gain = 0.1;  % Controller Gain
Error = eps+1; % Initial Error

% Frame Rotations
rx = -pi/2;
ry = 0;
rz = pi;

Rx = cos(rx/2) + sin(rx/2)*DQ.i;
Ry = cos(ry/2) + sin(ry/2)*DQ.j;
Rz = cos(rz/2) + sin(rz/2)*DQ.k;

X_ra = Rx*Ry*Rz;  % Frame Transformation from Robot to Arm Space
vel = 10;  % Joint velocity (percentage)
% Sequence 1
while norm(Error) > eps
    Jacob = ArmDQ.jacobian(Theta);  % Arm Joint Positions Jacobian
    Xm = ArmDQ.fkm(Theta); % Current Position
    Xh_ant = Xm;
    Error = Xd.q-Xm.q; % Error Calculation
    Theta = Theta + pinv(Jacob)*Gain*Error; % Controller Algorithm
    Xh_pos = ArmDQ.fkm(Theta); % Updated Position
    X_error = pinv(Xh_ant) .*  Xh_pos;  % Displacement of the Arm
    Xrm = ax.getjoints;
    Xrm = AX18DQ.fkm(Xrm);
    Xr_ant = Xrm*(cos(pi/2)+DQ.i*sin(pi/2)); % Current Robot Arm Position
    JacobR = AX18DQ.jacobian(Joint);
     X_error = (1+DQ.E*translation(X_error))*P(X_error);
    Xr_pos = Xr_ant .* (X_ra * X_error); % Robot Position Updating
    Xr_dis = translation(Xr_pos) - translation(Xr_ant);
    Joint = Joint + dinv(AX18DQ.jacobp(JacobR,Xr_ant),0.05)*0.3*vec4(Xr_dis);
    % Updates position
    Joint1 = (Joint*180)/pi;
    for i=1:5
        ax.joint(i,Joint1(i),vel)
    end
    %Joint=(ax.getjoints)*(pi/180); % Robot Arm Joints Current Position
    % Graphical Position Updating
%     subplot (1,2,1)
%     plot(ArmDQ, Theta'); % Arm
%     subplot (1,2,2)
%     plot(AX18DQ,Joint);  % Robot Arm
    %pause()
    pause(0.1)
end
% 
% pause()
% % Sequence 2
% 
% Xd = ArmDQ.fkm(Theta0);  % Final (Desired) Position
% 
% Error = eps+1; % Initial Error
% 
% 
% while norm(Error) > eps
%     Jacob = ArmDQ.jacobian(Theta);  % Arm Joint Positions Jacobian
%     Xm = ArmDQ.fkm(Theta); % Current Position
%     Xh_ant = Xm;
%     Error = Xd.q-Xm.q; % Error Calculation
%     Theta = Theta + pinv(Jacob)*Gain*Error; % Controller Algorithm
%     Xh_pos = ArmDQ.fkm(Theta); % Updated Position
%     X_error = pinv(Xh_ant) .*  Xh_pos;  % Displacement of the Arm
%     Xr_ant = getDualPosition(comau)*(cos(pi/2)+DQ.i*sin(pi/2)); % Current Robot Arm Position
%     JacobR = RobotDQModel.jacobian(Joint);
%      X_error = (1+DQ.E*translation(X_error))*P(X_error);
%     Xr_pos = Xr_ant .* (X_ra * X_error); % Robot Position Updating
%     Xr_dis = translation(Xr_pos) - translation(Xr_ant);
%     comau.setJointPositions(Joint*180/pi); % Sends Update Position Command to the Robot Arm
%     Joint=comau.getJointPositions()*(pi/180); % Robot Arm Joints Current Position
%     % Graphical Position Updating
%     subplot (1,2,1)
%     plot(ArmModel, Theta'); % Arm
%     subplot (1,2,2)
%     plot(RobotArm, [Joint;0]');  % Robot Arm
% end
