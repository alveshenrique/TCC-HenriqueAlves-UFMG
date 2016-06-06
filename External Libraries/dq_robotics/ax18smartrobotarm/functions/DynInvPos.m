function p = DynInvPos(L,H)
%DYNINVPOS computes the position in degrees from Dynamixel position
%bytes high and low
p = L + H*256;
p = ((300*pi/180)*p)/1023-(150*pi/180);