%================================ distmapE2 ==============================
%
% @namespace    constraints
% @class        distmapE2
%
% @brief        Impose constraints in the form of a look-up table
%
%
% In some cases, we won't have a precise description of the obstacles,
% instead we'll have some kind of gridded, binary map indicating the
% existence of an obstacle.  For these, cases, there needs to be a way to
% incorporate the numerical obstacle space into the optimization.  This
% class does just that.
%
% Given a distance map specified using a matrix, will impose distance
% constraints on the trajectory.  The E2 means Eucliadean 2-space (e.g.,
% Euclidean plane).
%
%================================ distmapE2 ==============================

%
% @name     distmap.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/18  [created]
%
% @note
%   set indent to 2 spaces.
%   set tabs to 4 spaces, with alignment and conversion to.
%
%================================ distmapE2 ==============================
classdef distmap < constraint


properties
  x;            %< Grid along x coordinate.
  y;            %< GRid along y coordinate.
  M;            %< The actual map itself as a matrix.

  Mx;           %< The gradient in x direction of the map.
  My;           %< The gradient in y direction of the map.
end


methods

   function this = distmap(xg, yg, M)

   this.x = xg;
   this.y = yg;

   this.M = M;

   this.build();

   end


end

methods (Access=protected)

   function build(this)

   % Compute gradients.
   % Populate in Mx and My.

   end

end


methods(Static)

  % Helper function to generate distmap from binary map.

  % Helper function to generate a capped sistmap from binary map.

end

%
%================================ distmapE2 ==============================
