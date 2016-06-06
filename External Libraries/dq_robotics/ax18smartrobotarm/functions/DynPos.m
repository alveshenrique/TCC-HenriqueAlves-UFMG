function [L H] = DynPos(p)
%DYNPOS returns the position of a Dynamixel device in 2 bytes for writing
%in register
%   [L H] = DYNPOS(p) returns the position bytes low 'L' and high 'H'
%   corresponding to position 'p' (from 0 to 1023)

H = dec2hex(bitand(768,p));
H = hex2dec(H(1));
L = bitand(255,p);
