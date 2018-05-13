%================================ hilare01 ===============================
%
% @brief    Simple script to demonstrate functionality of optgragen wrapper
%           for planar SE objects. (position + orientation)
% 
%================================ hilare01 ===============================

%
% @file     hilare01.m
%
% @author   Patricio A. Vela,       pvela@gatech.edu
% @date     2016/08/11 [created]
%
% @note
%   indent is 2 spaces.
%   tab is 4 spaces, with conversion.
%
%================================ hilare01 ===============================

%==[0] Setup the environment.
MATLIBS = { 'control/optimal/Optragen/src' , ...
            'control/optimal/snopt/', ...
            'control/optimal/snopt/matlab/matlab/'};
ivalab.loadLibraries( MATLIBS );


%==[1] Setup the problem.
tspan = [0, 6];

xSym = { 'x' , 'y', 'th' };
uSym = { 'u1', 'u2' };

xParms = { {4, 2, 3} ;  {4, 2, 3} ; {4, 2, 3} };
xTraj = traj.vector2traj( xSym, xParms );

uParms = { {4, 1, 1} ; {4, 1, 1} ; {4, 1, 1} };
uTraj = traj.vector2traj( uSym, uParms );

optProb = optmodels.planarSE(xTraj, uTraj);

%
%================================ hilare01 ===============================
