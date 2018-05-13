%============================ getTrajSplines ============================
%
% function xSP = getTrajSplines(nlp,x);
%
%
% @brief    Convert optimization variables into spline functions.
%
% @param[in]    nlp     Nonlinear programming problem structure.
% @param[in]    x       Solution to optimization problem.
%
% @param[out]   xSP     Cell array of splined trajectories from solution.
%
%============================ getTrajSplines ============================

%
% @file     getTrajSplines.m
%
% @author   Raktim Bhattacharya,    raktim@aero.tamu.edu
% @date     2005
%
% @note
%   indent is 2 spaces.
%   tab is aligned- and converted- to 4 spaces.
%
% Original notes in function:
%
%   OPTRAGEN
%   Copyright (c) 2005 by
%   Department of Aerospace Engineering
%   Texas A&M University.
%   All right reserved.
% =======================================================================
function xSP = getTrajSplines(nlp,x);

xSP = [];
count = 0;
for i=1:nlp.nout
  ni = nlp.ninterv(i);
  ord = nlp.order(i);
  sm = nlp.smoothness(i);
  ncoef = ni*(ord-sm) + sm;
  X = x((count+1):(ncoef+count));
  knots = linspace(0,nlp.HL,ni+1);
  augknots=augknt(knots,ord,ord-sm);
  xSP{i} = spmak(augknots,X');
  count = count + ncoef;
end
    
    
    
