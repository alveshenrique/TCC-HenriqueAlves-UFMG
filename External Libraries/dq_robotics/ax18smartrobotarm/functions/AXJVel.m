function AXJVel(s,j,v)
% AXJOINT moves AX18 Smart Robot Arm joint 'j' (1 to 5) to position 'p' (in degrees) through
% communication port 's'
[Lvel Hvel] = DynLH(v);
if j==1
    DynWrite(s,1,32,[Lvel Hvel]);
end
% if j==2
%     [Lpos1 Hpos1] = DynPos(1023-Pos);
% 
%     DynRegWrite(s,2,32,[Lpos Hpos Lvel Hvel]);
%     DynRegWrite(s,3,30,[Lpos1 Hpos1 Lvel Hvel]);
% 
%     DynAction(s,254);
% end
% if j==3
%     Pos1 = fix((1023*(p+150))/300);  % Desired position 0 to 1023
%     [Lpos Hpos] = DynPos(Pos1);
%     [Lpos1 Hpos1] = DynPos(1023-Pos);
%     [Lvel Hvel] = DynLH(v);
% 
%     DynRegWrite(s,4,30,[Lpos1 Hpos1 Lvel Hvel]);
%     DynRegWrite(s,5,30,[Lpos Hpos Lvel Hvel]);
% 
%     DynAction(s,254);
% end
% if j==4    
%     DynWrite(s,6,30,[Lpos Hpos Lvel Hvel]);
% end
% if j==5
%     DynWrite(s,7,30,[Lpos Hpos Lvel Hvel]);
% end