function AxJoint4(s,p)
%AXJOINT1 sets the AX18 Smart Robot Arm joint 4 position
%   AXJOINT1(s,p) Sets the AX18 Smart Robot Arm joint 4 position 'p' (in degrees, range -150 to 150), through
%   communication port s

Pos = fix((1023*(p+150))/300);  % Desired position 0 to 1023
id = 6;  % Device ID

Hpos = dec2hex(bitand(768,Pos));
Hpos = hex2dec(Hpos(1));
Lpos = bitand(255,Pos);

DynWrite(s,id,30,[Lpos Hpos]);



