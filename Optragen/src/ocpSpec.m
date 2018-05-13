%================================ ocpSpec ================================
%
% @class        ocpSpec
% @brief        Optimal control specification base class.
%
%
% Used to define the generic interface for specifications.  This class 
% does nothing, rather it is used to define the interface for all 
% subclasses.
%
%
%================================ ocpSpec ================================

%
% @file     ocpSpec.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/13  [created]
%
% @note
%   indent is 2 spaces.
%   tab is aligned - and converted- to 4 spaces.
%
%================================ ocpSpec ================================
%
%(
classdef ocpSpec < handle

%============================ Member Variables ===========================
%
%(
properties
  func;             %! Contains structural form of specification.
end

%)
%
%============================= Public Methods ============================
%
%(

methods

  %============================== ocpSpec ==============================
  %
  % @brief      Constructor for optimal control problem specification.
  %
  function this = ocpSpec()

  this.func = [];

  end

  %================================ get ================================
  %
  % @brief  Get asset properties from the specified object and return value.
  %
  %
  function val = get(a,prop_name)

  val = [];

  end

  %================================ set ================================
  %
  % @brief  Set asset properties and return the updated object
  %
  %
  function a = set(a,varargin)
  
  end

end


%)
%
%=========================== Protected Methods ===========================
%
%(

methods(Access = protected)

  %============================= cFunction =============================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  %TODO: Above description wrong.
  %
  function spec = cFunction(this)

  spec.grad = [];
  spec.func = [];
  spec.Tnames = [];

  end

  %============================= mFunction =============================
  %
  % @brief  Defines constraint function as an m-function.
  %
  %
  function spec = mFunction(this)

  %TODO:  I THINK THAT CFUNCTION AND MFUNCTION GOT FLIPPED!!
  %TODO:  Need to move cFunction stuff to here and code new c-function
  %TODO:    interface.  Could c-function refer to mex functions?

  spec.grad = [];
  spec.func = [];
  spec.Tnames = [];

  end


  %============================== charSpec =============================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  function spec = charSpec(this)
  
  spec.grad = [];
  spec.func = [];
  spec.Tnames = [];

  end

end

%)
%

end
%)
%
%================================ ocpSpec ================================
