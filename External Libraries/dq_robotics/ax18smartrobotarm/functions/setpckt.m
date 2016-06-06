function data = setpckt(id,inst,params)
%SETPCKT builds a Dynamixel instruction packet
%   SETPCKT(id,inst,params) builds a Dynamixel packet to the 'id' address,
%   with the 'inst' instruction of 'params' parameters. Returns the
%   response data.
%   'id' is an address from 1 to 255, 254 is the broadcast address
%   'inst' is an instruction from 0 to 18 for EEPROM insrtruction, and from
%   24 to 49 for RAM instruction

% Ernesto Lana
% Version 1.0, date: July, 2012





