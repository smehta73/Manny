%================================== traj =================================
%
% @class    traj
%
% @brief    Trajectory specification class for trajectory generation.
%
% 
%================================== traj =================================

%
% @file     traj.m
%
% @author   Raktim Bhattacharya,    raktim@tamu.edu
%           Patricio A. Vela,       pvela@gatech.edu (re-worked original)
% @date     2004 [created]
%
% @note
%   indent is 2 spaces.
%   tabs are 4 spaces, with conversion.
%
% Originally from:
%
%   Nonlinear Path Planning Toolbox v 1.0
%   Copyright (c) 2004 by                
%   Raktim Bhattacharya
%   California Institute of Technology               
%   Control and Dynamical Systems 
%   All right reserved.                
%
%================================== traj =================================
%
%(
classdef traj < handle


%============================ Member Variables ===========================
%
%(
properties (GetAccess = public, SetAccess = protected)
  name;
  nIntervals;
  oSmooth;
  order;

  nDeriv;
  derivOf;
end

%)
%
%============================= Public Methods ============================
%
%(

%----------------------- Constructor + Basic Info ------------------------
%
%--(

methods

  %================================ traj ===============================
  %
  % @brief      Constructor for the trajectory approximator.
  %
  % Invoking as per,
  %
  %  T = traj(nInterv, oSmooth, oApprox) 
  %
  %  creates a Trajectory object from the argument list.
  %
  %
  % @param[in]  vSym        Symbolic label for variable [string].
  % @param[in]  nInterv     Number of intervals to use.
  % @param[in]  oSmooth     Order of smoothness constraints. 
  % @param[in]  oApprox     Order of approximating polynomial.
  %
  function T = traj(vSym, nInterv, oSmooth, oApprox)

  if (nargin ~=4)
    error('Usage: T = traj(name, nInterv, oSmooth, oApprox)');
  end

  T.name    = vSym;
  T.nIntervals = nInterv;
  T.oSmooth = oSmooth;
  T.order   = oApprox;

  T.nDeriv  = 0;
  T.derivOf = []; 

  end

  %================================ get ================================
  %
  % @brief  Get asset properties from the specified object and return 
  %         the value.
  %
  % @param[in]  pName   Name of the member field/property to get.
  % @param[out] pVal    Value of the member field/property.
  %
  function pVal = get(this, pName)
  
  switch pName
    case {'name','variable'}
      pVal = this.name;
    case {'ninterv','nIntervals'}
      pVal = this.nIntervals;
    case {'smoothness'}
      pVal = this.oSmooth;
    case {'order','oApprox'}
      pVal = this.order;
    case {'nderiv','nDeriv'}
      pVal = this.nDeriv;
    case {'derivof', 'derivOf'}
      pVal = this.derivOf;
    otherwise
      error([pName,' Is not a valid asset property']);
  end

  end

  %================================ set ================================
  %
  % @brief  Set asset properties of the specified object. 
  %
  % Can specify multiple name / value pairs.  At least one pair is
  % needed.
  %
  % @param[in]  pName   Name of the member field/property to set.
  % @param[in]  pVal    Value of the member field/property to set.
  %
  function set(this, pName, pVela, varargin)

  propertyArgs = varargin;
  moreArgs = true;

  while moreArgs
    switch pName

      case {'ninterv','nIntervals'}
        if ( isnumeric(pVal) && (size(pVal) == [1,1]) )
          this.nIntervals = uint32(pVal);
        else
          error('Scalar integer expected');
        end

      case {'smoothness','oSmooth'}
        if ( isnumeric(val) && (size(val) == [1,1]) )
          this.oSmooth = uint32(val);
        else
            error('Scalar integer expected');
        end

      case {'order','oApprox'}
        if ( isnumeric(val) && (size(val) == [1,1]) )
          this.order = uint32(val);
        else
          error('Scalar integer expected');
        end

% IS NEEDED?
%      case 'nderiv'
%        error('Cannot change nderiv property directly');
%      case 'derivof'
%        error('Cannot change derivof property directly');
%      otherwise
%        error('Asset properties: ninterv,mult,order')
    end

    if (length(propertyArgs) >= 0)
      pName = propertyArgs{1};
      pVal  = propertyArgs{2};
      propertyArgs = propertyArgs(3:end);
    else
      moreArgs = false;
    end

  end

  end

%--)
%
%------------------------------- Operations ------------------------------
%
%--(


  %=============================== deriv ===============================
  %
  % @brief  Returns derivative object of thhe trajectory approximator.
  %
  function Td = deriv(T, dname);

  if ~isa(T,'traj')
    error('First argument must be a trajectory object');
  end

  Td = traj(dname, [], [], []);

  Td.nDeriv = T.nDeriv + 1;

  if (T.nDeriv == 0)
    Td.derivOf = T.name;
  else
    Td.derivOf = T.derivOf;
  end

  end

%--)
%
%------------------------------- Display -------------------------------
%
%--(


  %============================== display ==============================
  %
  %  @brief     Displays a trajectory object.
  %
  function display(this)
  
  [m,n] = size(a);

  if ( (m > 1) || (n > 1) )
    disp(['Trajectory matrix of size [' num2str([m,n]) ']']);
  elseif isempty(a.derivof)
    derivof = '<none>';
    stg = sprintf([' ninterv: %d\n smoothness: %d\n order: %d\n', ...
                   ' nderiv: %d\n derivof: %s'], ...
                  this.nIntervals, this.oSmooth, this.order, ...
                  this.nDeriv, this.derivof);
  else
    stg = sprintf(' nderiv: %d\n derivof: %s', this.nderiv, this.derivof);
  end

  disp(stg);

  end
    

%--)

end

%)
%
%============================= Static Methods ============================
%
%(


methods(Static)

  %=========================== vector2traj =============================
  %
  % @brief      Convert symbolic vector into trajectory vector.
  %
  % @param[in]  xSym    Vector of symbolic names [strings in cells].
  % @param[in]  iParms  Instantiation parameters per element in xSym [cells].
  %
  function tVec = vector2traj(xSym, iParms)

  assert( size(xSym,1) == size(iParms,1) )

  for ii=1:size(xSym,1)
    tVec(ii) = traj(xSym{ii} , iParms{ii}{:});
  end

  end

  %============================== parmCell =============================
  %
  % @brief      Create cell vector from traj initialization parameters.
  %
  function pc = parmCell(vSym, nInterv, oSmooth, oApprox)

  pc = { vSym, nInterv, oSmooth, oApprox };

  end

  %================================ plus ===============================
  %
  % @brief      Join two trajectory objects.
  %
  %TODO: Not really sure how used! NEED TO FIGURE OUT.
  %
  function T = plus(T1,T2)

  [m1,n1] = size(T1);
  [m2,n2] = size(T2);
 
  if (m1~=1 | m2~=1)
    error('Arguments must be row vectors');
  end

  T = [T1 T2];

  end

  %============================== trajList =============================
  %
  % @brief      Not sure what this does.
  %
  function ret = trajList(varargin)
  
  ret = [];
  for i = 1:nargin
    if (~isa(varargin{1},'traj'))
      error('Only trajectory object are valid input arguments');
    end
    
    if size(varargin{1}) ~= [1 1]
      error('Only scalar trajectory objects are allowed');
    end 
    ret(i).varname = inputname(i);
    ret(i).traj    = varargin{i};
  end

   List = {ret.varname};
   uniqueList = unique(List);
 
   if length(List)~= length(uniqueList)
     error('Only unique trajectory objects allowed in argument');
   end
   
   end

end

%)
%

end

%)
%
%================================== traj =================================
