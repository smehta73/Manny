%============================== createfooter =============================
%
% @brief    Create the footer content for the optimal control interface 
%           function object.
%
%============================== createFooter =============================

%
% @file     createFooter.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/11 [created]
%
% @note
%   indent is 2 spaces.
%   tabs are 4 spaces, with conversion.
%
%============================== createFooter =============================
function createFooter(fptr);

fprintf(fptr, '\nend');
fclose(fptr);

end
%
%============================== createFooter =============================
