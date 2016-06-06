function AXJoint(s,j,p,v)
% AXJOINT moves AX18 Smart Robot Arm joint 'j' (1 to 5) to position 'p' (in
% radians) at velocity 'v' (0 to 1) through communication port 's'
if j == 2
    p = -(p+pi/2);
end
if j == 3
    p = -p;
end
Pos = fix(1023*((p+(150*pi/180))/(300*pi/180)));  % Desired position 0 to 1023
if Pos <0
    Pos=0;
elseif Pos>1023
    Pos = 1023;
end
[Lpos Hpos] = DynLH(Pos);
vel = fix(v*1023);
if vel==0
    vel=1;
end
[Lvel Hvel] = DynLH(vel);
if j==1
    DynWrite(s,1,30,[Lpos Hpos Lvel Hvel]);
end
if j==2
    [Lpos1 Hpos1] = DynLH(1023-Pos);

    DynRegWrite(s,2,30,[Lpos Hpos Lvel Hvel]);
    DynRegWrite(s,3,30,[Lpos1 Hpos1 Lvel Hvel]);

    DynAction(s,254);
end
if j==3
    [Lpos1 Hpos1] = DynPos(1023-Pos);

    DynRegWrite(s,4,30,[Lpos1 Hpos1 Lvel Hvel]);
    DynRegWrite(s,5,30,[Lpos Hpos Lvel Hvel]);

    DynAction(s,254);
end
if j==4    
    DynWrite(s,6,30,[Lpos Hpos Lvel Hvel]);
end
if j==5
    DynWrite(s,7,30,[Lpos Hpos Lvel Hvel]);
end