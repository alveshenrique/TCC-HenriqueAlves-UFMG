%ISVEC Test if argument is a homogeneous transformation
%
%	H = ISVEC(V, l)
%
%  Returns true (1) if the argument tr is of dimension 4x4.

% Copyright (C) 2002-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function h = isvec(v, l)
    if nargin == 1,
            l = 3;
    end
	if min(size(v)) == 1,
		h =  length(v) == l;
	else
		h = 0;
	end
