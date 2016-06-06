classdef orientationController < handle
    %ORIENTATIONCONTROLLER Defines the object for
    %the position priority controller for robot manipulators
    %   orientationController(model,kp,ko) defines the object
    %   for the position priority controller of the robot
    %   manipulator with dual quaternions kinematic model. 'model' is the
    %   DQ kinematic model of the manipultor, 'kp' is either a 8x8
    %   square gain matrix or a scalar related to position and 'ko'
    %   is analogous to kp related to orientation. The controller gives
    %   priority to position control and performs orientation control in
    %   the null space of the task.
    %   Dependencies: DQ toolbox by Bruno Adorno
    %   Properties: DQ kinematic model, Controller's position gain matrix
    %   Kp, Controller's orientation gain matrix Ko
    
    %<<<<<<<<<IMPLEMENT!!!!!!>>>>>>>>>>>>>
    
    properties
        DQmodel
        Ko
    end
    
    methods
        function controller = orientationController(model,ko)
            
            % Assigns the DQ model to the robot manipulator
            if strcmp(class(model),'DQ_kinematics') == 1 % Verifies if model is a valid DQ kinematic model
                controller.DQmodel = model;
                % Assigns the orientation matrix gain Ko to the controller
                if numrows(ko) == 8 && numrows(ko) == 8  % Verifies conditioning of ko
                    controller.Ko = ko;  % Matrix Ko
                elseif numel(ko) == 1
                    controller.Ko = ko;   % Scalar ko
                else
                    error('orientationController:orientationController:NotValidGainMatrix','ko must be either a 8x8 square matrix\n or a scalar')
                end
            else
                error('orientationController:orientationController:DQmodelNotValid','Not a valid DQ kinematic model')
            end
        end
        function theta = getUpdatedJointPosition(controller,xd,prev_theta)
            xm = controller.DQmodel.fkm(prev_theta); % Current dual position
            J = controller.DQmodel.jacobian(prev_theta); % Jacobian matrix for joint position prev_theta
            Jp = controller.DQmodel.jacobp(J,xm); % Position Jacobian matrix
            P = eye(5) - pinv(J)*J;  % Null space projection of the Jacobian matrix
            errorp = translation(xd) - translation(xm);  % Position error
            erroro = rotation_axis(xd) - rotation_axis(xm);  % Orientation error
            theta = prev_theta + pinv(Jp)*controller.Kp*vec4(errorp) + P*pinv(Jp)*controller.Ko*vec4(erroro); % Controller algorithm
        end
    end
    
end

