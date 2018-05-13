%============================== createHeader =============================
%
% @brief    Create the header content for the optimal control interface 
%           function object.
%
%============================== createHeader =============================

%
% @file     createHeader.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/11 [created]
%
% @note
%   indent is 2 spaces.
%   tabs are 4 spaces, with conversion.
%
%============================== createHeader =============================
function fptr = createHeader(funcName, pathName);

if (nargin > 1)
  fname = [pathName '/' funcName '.m'];
else
  fname = [funcName '.m'];
end

fptr = fopen(fname,'w');
if (fptr == -1)
  error(['Cannot create ' fname '.']);
end

fprintf(fptr, ['function och = ' funcName '()\n\n']);
fprintf(fptr, ['och.icf = @' funcName '_icf; \n']);
fprintf(fptr, ['och.tcf = @' funcName '_tcf; \n']);
fprintf(fptr, ['och.fcf = @' funcName '_fcf; \n\n']);
fprintf(fptr, ['och.nlicf = @' funcName '_nlicf; \n']);
fprintf(fptr, ['och.nltcf = @' funcName '_nltcf; \n']);
fprintf(fptr, ['och.nlfcf = @' funcName '_nlfcf; \n']);
fprintf(fptr, ['och.nlgcf = @' funcName '_nlgcf; \n']);
fprintf(fptr, '%=================== Interface Functions ===================');
fprintf(fptr, '\n\n');

end
%
%============================== createHeader =============================
