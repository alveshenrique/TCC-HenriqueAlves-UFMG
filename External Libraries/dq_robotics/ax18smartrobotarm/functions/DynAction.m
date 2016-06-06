function DynAction(s,id)
%DYNACTION executes a command previously sent using the DYNREGWRITE
%fuction.
%   DYNACTION(s,id) sends an execution packet for a previously stored command 
%   in device with ID 'id' through the communcation port 's'.
%   'id' is the device ID from 0 to 253, 254 is the broadcast address.

% Ernesto Lana
% Version 1.0
% July, 2012

ln = 2;  % Length of data packet
inst = 5; % Instruction, 5 = action

chksm = DynChecksum(id,ln,inst,0);  % Computes the packet checksum

pckt = [255 255 id ln inst chksm];  % Builds the dynamixel packet

fwrite(s,pckt)  % Writes the dynamixel packet to the specified device

% Clears the data buffer
if s.BytesAvailable ~= 0
    fread(s,s.BytesAvailable);
end
