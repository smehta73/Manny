%=============================== constraint ==============================
%
% @class    constraint
%
% @brief    Implements abstraction of an optimal control constraint.
%
% 
%=============================== constraint ==============================

%
% @file     constraint.m
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
%=============================== constraint ==============================
%
%(
classdef constraint < ocpSpec

%============================ Member Variables ===========================
%
%--(
properties
  type;                 %! Type of constraint.
end

%--)
%
%============================= Public Methods ============================
%
%--(

methods

  %============================= constraint ============================
  %
  % @brief      Define a constraint for inclusion into optimal control problems.
  %
  % This function defines the initial cost function
  % func definition can be either
  %           a. Character string containing the function
  %           b. mFunction object
  %           c. cFunction object
  %
  %TODO: Should move different types to their own classes?
  %
  function this = constraint(lb,func,ub,type, xVars)

  this@ocpSpec();

  %================== Error Checking for Input Data ==================
  if nargin~=5
    error('Usage: constrObj = constraint(lb,func,ub,type, xVars);');
  end

  if ~isa(lb,'numeric')
    error('Expecting numeric array for lower bound');
  end

  if ~isa(ub,'numeric')
    error('Expecting numeric array for upper bound');
  end

  if ~isa(type,'char')
    error('Expecting character array for type');
  end

  TYPES = {'initial','trajectory','final','galerkin'};
  ii = strmatch(lower(type),TYPES,'exact');
  if isempty(ii)
    error(['Constraint <type> must be one of the following: ' ...
           'initial, trajectory, final, galerkin']);
  end

  %===================================================================

  this.type = lower(type); 

  if isa(func,'char')
    constr = this.charSpec(lb,func,ub, xVars);
  elseif isa(func,'mFunction')
    constr = this.mFunction(lb,func,ub);
  elseif isa(func,'cFunction')
    constr = this.cFunction(lb,func,ub);
  else
    error('Illegal data type in first argument.');
  end

  this.func = constr; 

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
  disp(['Cannot alter attributes of a constraint object. '
        'Create a new constraint object instead']);

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

%--)
%
%=========================== Protected Methods ===========================
%
%--(

methods(Access = protected)

  %============================= cFunction =============================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  %TODO: Above description might be wrong.
  %TODO: Don't really understand what code does. As in, what is
  %TODO:   get(func, 'varList') doing?
  %
  function constr = cFunction(this, lb,func,ub)

  % Get all the trajectories from the workspace

  N = get(func,'nFunc');

  if length(lb) ~= N | length(ub) ~= N
    error('Bounds must be scalars for analytical constraint functions');
  end

  % Check signature list of the cFunction
  % It should match that required by nonlinear constraints

  constr.grad = [];
  constr.func = func;
  constr.Tnames = get(func,'varList');
  constr.lb = scaleInf(lb);
  constr.ub = scaleInf(ub);

  end

  %============================= mFunction =============================
  %
  % @brief  Defines constraint function as an m-function.
  %
  %
  function constr = mFunction(this, lb, func, ub)

  %TODO:  I THINK THAT CFUNCTION AND MFUNCTION GOT FLIPPED!!
  %TODO:  Need to move cFunction stuff to here and code new c-function
  %TODO:    interface.  Could c-function refer to mex functions?

  end


  %============================== charSpec =============================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  function constr = charSpec(this, lb, func, ub, Tnames)
  
  % Get all the trajectories from the workspace
  
  if length(lb) > 1 | length(ub) > 1
    error('Bounds must be scalars for analytical constraint functions');
  end
  
  varnames = symvar(func);
  
  fvars = [];
  for i=1:length(varnames)
    I = strcmp(varnames{i},Tnames);
    if sum(I)>0 
      fvars = [fvars,{varnames{i}}];
    end
  end
  
  grad = optrautil.getGradient(func,fvars);   % Char cell array
  
  constr.grad = grad;
  constr.func = func;
  constr.Tnames = fvars;
  constr.lb = scaleInf(lb);
  constr.ub = scaleInf(ub);

  end

end

%--)
%
%============================= Static Methods ============================
%
%--(

methods(Static)

  %========================== initialCondition =========================
  %
  % @brief  Specify initial condition constraints.
  %
  % @param[in]  vSym    Symbolic string of variable.
  % @param[in]  vVal    Numerical value of initial condition.
  %
  function ic = initialCondition(vSym, vVal)
  ic = constraint(vVal, vSym, vVal, 'initial', vSym);
  end

  %=========================== finalCondition ==========================
  %
  % @brief  Specify final condition constraints.
  %
  % @param[in]  vSym    Symbolic string of variable.
  % @param[in]  vVal    Numerical value of initial condition.
  %
  function ic = finalCondition(vSym, vVal)
  ic = constraint(vVal, vSym, vVal, 'final', vSym);
  end

  %=============================== limits ==============================
  %
  % @brief  Specify known limits on optimization variables.
  %
  % @param[in]  vSym    Symbolic variable to limit/constraint.
  % @param[in]  bLims   Scalar or 2-vector specifying lower/upper limits.
  %
  function lc = limits(vSym, bLimits)
  lc = constraint(bLimits(1), vSym, bLimits(2), 'trajectory', vSym);
  end

  %============================= obsCircle =============================
  %
  % @brief  Create a circular (obstacle) constraint.
  %
  % @param[in]  xSym    Symbolic description of state vector.
  % @param[in]  xCent   Center of circle (same dimensions as xSym).
  % @param[in]  rad     Radius constraint (must be greater than).
  %
  function oc = obsCircle(xSym, xCent, rad)
  nSqStr = [];
  for ii = 1:length(xSym)
    nSqStr = [nSqStr , ['(' xSym{ii} '- (' num2str(xCent(ii)) ') )^2'] ];
    if (ii < length(xSym))
      nSqStr = [nSqStr  ' + '];
    end
  end
  oc = constraint(rad^2 , nSqStr, Inf, 'trajectory', xSym);

  end

  %========================== obsSquareApprox ==========================
  %
  % @brief  Create an approximately square obstacle constraint.
  %
  % @param[in]  xSym    Symbolic description of state vector.
  % @param[in]  xCent   Center of square/rectangular object.
  % @param[in]  rad     Radius constraint (scalar or 2-vector).
  % @param[in]  order   Order of Lp constraint approximating the sqyare.
  %
  function oc = obsSquareApprox(xSym, xCent, rad, order)
  nLpStr = [];

  order = round(order);         % Force to be natural.
  if (mod(order,2) == 1)        % Force to be even.
    order = order + 1;
  end
  oStr = num2str(order);

  for ii = 1:length(xSym)
    nLpStr = [nLpStr , ['( (' xSym{ii} '- (' num2str(xCent(ii)) ') )/' ...
              '(' num2str(rad) ') )^' oStr] ];
    if (ii < length(xSym))
      nLpStr = [nLpStr  ' + '];
    end
  end
  oc = constraint(1, nLpStr, Inf, 'trajectory', xSym);

  end

end

%--)
%

end
%)
%
%=============================== constraint ==============================
