function [L H] = DynLH(position)
%DYNLH returns the two bytes of a Dynamixel command
%   [L H] = DYNLH(p) returns the bytes low 'L' and high 'H'
%   corresponding to input 'p' (from 0 to 1023)

H = dec2hex(bitand(768,position));
H = hex2dec(H(1));
L = bitand(255,position);
