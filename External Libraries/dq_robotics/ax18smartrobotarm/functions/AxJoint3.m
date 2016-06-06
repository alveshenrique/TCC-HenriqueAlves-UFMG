function AxJoint3(s,p)
%AXJOINT1 sets the AX18 Smart Robot Arm joint 2 position
%   AXJOINT1(s,p) Sets the AX18 Smart Robot Arm joint 2 position 'p' (in degrees, range -90 to 90), through
%   communication port s

Pos1 = fix((1023*(p+150))/300);  % Desired position 0 to 1023

[Lpos1 Hpos1] = DynPos(Pos1);

[Lpos2 Hpos2] = DynPos(1023-Pos1);

DynRegWrite(s,4,30,[Lpos2 Hpos2]);
DynRegWrite(s,5,30,[Lpos1 Hpos1]);

DynAction(s,254);