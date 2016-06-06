function AxGrip(s,p)
%AXGRIP controls the gripper openness
%   AXGRIP(p) opens the AX-18 Smart Robot Arm gripper from 0 (closed) to
%   1 (open) positions, through communication port s


d = (208*p);  % Ammount of displacement of the gripper
Pos1 = fix(720-d); % Right actuator position

[Lpos1 Hpos1] = DynPos(Pos1);

Pos2 = fix(304+d);  % Right actuator position

[Lpos2 Hpos2] = DynPos(Pos2);


DynWrite(s,8,30,[Lpos1 Hpos1]);
DynWrite(s,9,30,[Lpos2 Hpos2]);
