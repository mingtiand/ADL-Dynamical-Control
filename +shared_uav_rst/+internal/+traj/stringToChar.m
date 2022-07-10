function charInputs = stringToChar(nargin,varargin) 
%This function is for internal use only. It may be removed in the future.
%charInputs Convert strings to chars case by case for codegen support

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    if nargin > 17
        charInputs = cell(1,16);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 15
        charInputs = cell(1,14);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 13
        charInputs = cell(1,12);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 11
        charInputs = cell(1,10);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 9
        charInputs = cell(1,8);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 7
        charInputs = cell(1,6);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 5
        charInputs = cell(1,4);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    elseif nargin > 3
        charInputs = cell(1,2);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    else
        charInputs = {};
    end
