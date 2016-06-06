function data = DynPing(s,id)
%DYNPING retieves a status packet from a Dynamixel device
%   DYNPING(s,id) sends a command to device with ID 'id' to retrieve a
%   status packet through communication port 's'.
%   'id' is the device ID from 0 to 253, 254 is the broadcast address

% Ernesto Lana
% Version 1.0
% July, 2012

ln = 2;  % Length of data packet
inst = 1; % Instruction, 1 = ping


chksm = DynChecksum(id,ln,inst,0);  % Computes the packet checksum

pckt = [255 255 id ln inst chksm];  % Builds the dynamixel packet

fwrite(s,pckt)  % Writes the dynamixel packet to the specified device

data = fread(s,6);  % Reads status packet coming from the device


