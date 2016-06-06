function Jinv = dinv( J,lmb )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Jinv = (J'*J+lmb*eye(size(J,2)))\J';
end

