classdef invariantIsotropicDampedController < handle
    %INVARIANTISOTROPICDAMPEDCONTROLLER Defines the object for
    %the invariant error metric damped controller for robot manipulators
    %   invariantIsotropicDampedController(model,k,lmb) defines the object
    %   for the invariant error metric damped controller of the robot
    %   manipulator with dual quaternions kinematic model. 'model' is the
    %   DQ kinematic model of the manipultor, 'k' is either a 8x8
    %   square gain matrix or a scalar and 'lmb' is the scalar
    %   damping factor
    %   Dependencies: DQ toolbox by Bruno Adorno
    %   Properties: DQ kinematic model, Controller's gain matrix K and damping
    %   factor lambda
    
    properties
        DQmodel
        K
        lambda
    end
    
    methods
        function controller = invariantIsotropicDampedController(model,k,lmb)
            
            % Assigns the DQ model to the robot manipulator
            if strcmp(class(model),'DQ_kinematics') == 1 % Verifies if model is a valid DQ kinematic model
                controller.DQmodel = model;
                
                % Assigns the matrix gain K to the controller
                if numrows(k) == 8 && numrows(k) == 8  % Verifies conditioning of k
                    controller.K = k;  % Matrix K
                elseif numel(k) == 1
                    controller.K = k;   % Scalar k
                else
                    error('invariantIsotropicDampedController:invariantIsotropicDampedController:NotValidGainMatrix','k must be either a 8x8 square matrix\n or a scalar')
                end
                
                if numel(lmb) == 1
                    controller.lambda = lmb;  % Assigns the damping factor to the controller
                else
                    error('invariantIsotropicDampedController:invariantIsotropicDampedController:NotScalarLambda','Damping factor lambda must be a scalar')
                end
                
            else
                error('invariantIsotropicDampedController:invariantIsotropicDampedController:DQmodelNotValid','Not a valid DQ kinematic model')
            end
        end
        function theta = getUpdatedJointPosition(controller,xd,prev_theta)
            J = controller.DQmodel.jacobian(prev_theta); % Jacobian matrix for joint position prev_theta
            N = haminus8(xd)*diag([1,-1,-1,-1,1,-1,-1,-1])*J; 
            xm = controller.DQmodel.fkm(prev_theta); % Current dual position
            error = vec8(1-xm'*xd);  % Invariant error
            theta = prev_theta + dinv(N,controller.lambda)*controller.K*error;  % Controller algorithm
        end
    end
    
end

