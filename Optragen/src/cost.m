%================================== cost =================================
%
% @class    cost
%
% @brief    Implements abstraction of an optimal control constraint.
%
% 
%================================== cost =================================

%
% @file     cost.m
%
% @author   Raktim Bhattacharya,        raktim@ae.tamu.edu  [author]
%           Patricio A. Vela,           pvela@gatech.edu    [modify]
% @date     2004/XX/XX [created]
%
% @note
%   indent is 2 spaces.
%   tab is aligned- and converted to- 4 spaces.
%
% Original text in header:
%   Nonlinear Path Planning Toolbox v 1.0
%   Copyright (c) 2004 by                
%   Raktim Bhattacharya, (raktim@cds.caltech.edu)
%   California Institute of Technology               
%   Control and Dynamical Systems 
%   All right reserved.                
%
%================================== cost =================================
classdef cost < handle

%============================ Member Variables ===========================
%

properties
  func;                 %! Structure containing cost information.
  type;                 %! Type of cost.
end

%)
%
%============================= Public Methods ============================
%
%(

methods

  %================================ cost ===============================
  %
  % @brief      Constructor for cost function.
  %
  % This function defines the initial cost function
  % func definition can be either
  %           a) Character string containing the function
  %           b) mFunction object
  %           c) cFunction object
  %
  function this = cost(func,type)

  %=============== Error Checking for Input Data ===============

  if nargin~=2
    error('Usage: costObj = cost(function,type);');
  end

  if ~isa(type,'char')
    error('Expecting character array for type');
  end

  TYPES = {'initial','trajectory','final'};
  ii = strmatch(lower(type),TYPES,'exact');
  if isempty(ii)
    error(['Cost <type> must be one of the following: ' ...
           'initial, trajectory, final']);
  end

  %=============================================================

  this.type = lower(type); 

  if isa(func,'char')
    CI = this.cost_char(func);
  elseif isa(func,'mFunction')
    CI = this.cost_mFunction(func);
  elseif isa(func,'cFunction')
    CI = this.cost_cFunction(func);
  else
    error('Illegal data type in first argument.');
  end

  this.func = CI; 

  end


  %================================ get ================================
  %
  % @brief  Get asset properties from the specified object and return value.
  %
  %
  function val = get(a,prop_name)

  switch prop_name
    case 'func'
      val = a.func;
    case 'type'
      val = a.type;
    case 'tag'
      val = a.tag;
    otherwise
      error([prop_name,' Is not a valid asset property'])
  end

  end

  %================================ set ================================
  %
  % @brief  Set asset properties and return the updated object
  %
  %
  function a = set(a,varargin)
  
  property_argin = varargin;
  disp(['Cannot alter attributes of a cost object. '
        'Create a new cost object instead']);

  end

  %================================ plus ===============================
  %
  % @brief  Concatenate constraints.
  %
  function T = plus(T1,T2)

  [m1,n1] = size(T1);
  [m2,n2] = size(T2);

  if (m1~=1 | m2~=1)
    error('Arguments must be row vectors');
  end

  T = [T1 T2];

  end

end

%)
%
%=========================== Protected Methods ===========================
%
%(

methods (Access = protected)


  %=========================== cost_cFunction ==========================
  %
  % @brief  Defines cost function, defined as a character array.
  %
  %TODO: Above description might be wrong.
  %
  function cost = cost_cFunction(this, func)

  N = get(func,'nFunc');
  name = get(func,'name');

  if N~=1
    error(['Cost defined by ANSI C function (%s) should return ' ...
           'scalar values.'],name);
  end

  % Check signature list of the cFunction
  % It should match that required by nonlinear constraints

  CI.grad = [];
  CI.func = func;
  CI.Tnames = get(func,'varList');

  end

  %========================== constraint_char ==========================
  %
  % @brief  Defines cost function, defined as a character array.
  %
  function CI = cost_char(this, func)

  varnames = symvar(func);
  Tnames = getWorkSpaceTrajNames;

  fvars = [];
  for i=1:length(varnames)
    I = strcmp(varnames{i},Tnames);
    if sum(I)>0 
      fvars = [fvars,{varnames{i}}];
    end
  end

  grad = optrautil.getGradient(func,fvars);   % Char cell array

  CI.grad = grad;
  CI.func = func;
  CI.Tnames = fvars;

  end

end

%)
%
%=========================================================================

end

%
%================================== cost =================================
