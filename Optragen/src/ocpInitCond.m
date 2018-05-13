%============================== ocpInitCond ==============================
%
% @class        ocpInitCond
% @brief        Specify initial condition in optimal control problem.
%
%
%============================== ocpInitCond ==============================

%
% @file     ocpInitCond
%
% @author   Patricio A. Vela,   pvela@gatech.edu
% @date     2016/08/13  [created]
%
%============================== ocpInitCond ==============================
%
%(
classdef ocpInitCond < constraint


%============================= Public Methods ============================
%
%--(

methods

  %============================ ocpInitCond ============================
  %
  % @brief      Specify an initial condition constraint.
  %
  function this = ocpInitCond(vSym, vVal)

  this@constraint(vVal, vSym, vVal, 'initial');

  end

end

%--)
%

end
%)
%
%============================== ocpInitCond ==============================
