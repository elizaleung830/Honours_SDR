classdef sampOpt < uint16
    % An enumeration class for samples per sweep
    % The samples/sweep parameter is defined as a fraction of the maximum
    % sampling rate.
    enumeration
       one (0) 
       half (1) 
       oneFourth (2)
       oneEighth (3) 
    end
end