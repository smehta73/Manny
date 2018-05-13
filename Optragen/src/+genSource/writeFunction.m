%============================= writeFunction =============================
%
% @brief    Write the specified interface function for the optimal control 
%           interface function object.
%
%============================= writeFunction =============================

%
% @file     writeFunction.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/11 [created]
%
% @note
%   indent is 2 spaces.
%   tabs are 4 spaces, with conversion.
%
%============================= writeFunction =============================
function writeFunction(fptr, fname, codeStr);

fprintf(fptr,'\n%s\n',codeStr);

end
%
%============================= writeFunction =============================
