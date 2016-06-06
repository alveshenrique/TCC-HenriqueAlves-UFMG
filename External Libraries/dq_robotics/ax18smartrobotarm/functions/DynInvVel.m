function v = DynInvVel(L,H)
%DYNINVPOS computes the velocity n rpm from Dynamixel position
%bytes high and low
v = L + H*256;
if v>1023
    v = -(v-1024);
end
v = 0.111*v;
% Convert from rpm to rad/s
v= 2*v*pi/60;
