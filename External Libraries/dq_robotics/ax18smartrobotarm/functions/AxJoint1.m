function AxJoint1(s,p)
%AXJOINT1 sets the AX18 Smart Robot Arm joint 1 position
%   AXJOINT1(s,p) Sets the AX18 Smart Robot Arm joint 1 position 'p' (in degrees, range -150 to 150), through
%   communication port s

Pos = fix((1023*(p+150))/300);  % Desired position 0 to 1023
id = 1;  % Device ID

[Lpos Hpos] = DynPos(Pos);

DynWrite(s,id,30,[Lpos Hpos]);



