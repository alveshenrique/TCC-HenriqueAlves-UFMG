classdef positionController < handle
    %POSITIONCONTROLLER Defines the object for
    %the position controller for robot manipulators
    %   positionController(model,k) defines the object
    %   for the position controller of the robot
    %   manipulator with dual quaternions kinematic model. 'model' is the
    %   DQ kinematic model of the manipultor, 'k' is either a 8x8
    %   square gain matrix or a scalar
    %   Dependencies: DQ toolbox by Bruno Adorno
    %   Properties: DQ kinematic model, Controller's gain matrix K
    
    properties
        DQmodel
        K
    end
    
    methods
        function controller = positionController(model,k)
            
            % Assigns the DQ model to the robot manipulator
            if strcmp(class(model),'DQ_kinematics') == 1 % Verifies if model is a valid DQ kinematic model
                controller.DQmodel = model;
                
                % Assigns the matrix gain K to the controller
                if numrows(k) == 8 && numrows(k) == 8  % Verifies conditioning of k
                    controller.K = k;  % Matrix K
                elseif numel(k) == 1
                    controller.K = k;   % Scalar k
                else
                    error('positionController:positionController:NotValidGainMatrix','k must be either a 8x8 square matrix\n or a scalar')
                end
            else
                error('positionController:positionController:DQmodelNotValid','Not a valid DQ kinematic model')
            end
        end
        function theta = getUpdatedJointPosition(controller,xd,prev_theta)
            J = controller.DQmodel.jacobian(prev_theta); % Jacobian matrix for joint position prev_theta
            xm = controller.DQmodel.fkm(prev_theta); % Current dual position
            error = translation(xd) - translation(xm);  % Error
            theta = prev_theta + pinv(controller.DQmodel.jacobp(J,xm))*controller.K*vec4(error); % Controller algorithm
        end
    end
    
end

