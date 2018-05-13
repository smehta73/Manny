% =======================================================================
%   OCP2NLP
%   Copyright (c) 2005 by
%   Raktim Bhattacharya, (raktim@aero.tamu.edu)
%   Department of Aerospace Engineering
%   Texas A&M University.
%   All right reserved.
% =======================================================================
clear all;
global nlp;

addpath('../../');
addpath('../../src');
% SNOPTPATH = '../../../../snopt';
SNOPTPATH = '~/ivaMatlibs/control/snopt';
addpath([ SNOPTPATH ]);
addpath([ SNOPTPATH '/matlab/matlab/' ]);
addpath([ SNOPTPATH '/matlab/mex/' ]);

% Typesetting for figure text
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

ninterv = 2;
hl = 1.0;

l_1 = 0.5; l_2 = 0.5;   % link lengths

% initial/final end-effector poses
x0 = l_1 + l_2; 
y0 = 0; 
xf = 0;
yf = 0.5;

eps = 0.0001;

% Generate symbolic representation of end-effector pose
syms a1 a2      % a1 = joint 1 angle; a2 = joint 2 angle
g0_1 = [ cos(a1) -sin(a1) 0 ; ...
         sin(a1) cos(a1)  0 ; ...
         0 0 1 ];    % spatial frame to 1st link frame (co-located with first joint)
g1_2 = [ cos(a2) -sin(a2) l_1 ; ...
         sin(a2) cos(a2)  0 ; ...
         0 0 1 ];    % 1st link frame to 2nd link frame (co-located with second joint)
g2_3 = [ 1 0 l_2 ; ...
         0 1 0 ; ...
         0 0 1 ];    % 2nd link frame to end frame at end of 2nd link (ie. end-effector)
g_ee = g0_1*g1_2*g2_3;      % spatial frame to end-effector frame
g_ee_x_str = char(vpa(g_ee(1, 3), 9));      % end-effector x- spatial position
g_ee_y_str = char(vpa(g_ee(2, 3), 9));      % end-effector y- spatial position

% Create trajectory variablesbase
% ===========================
a1 = traj('a1', ninterv,2,3); % Arguments are ninterv, smoothness, order
a2 = traj('a2', ninterv,2,3);

% Create derivatives of trajectory variables
% ==========================================
a1d = deriv(a1, 'a1');
a2d = deriv(a2, 'a2');

ParamList = [];
xVars = {'a1'; 'a2'; 'a1d'; 'a2d'};

% Define constraints
% ==================
Constr = constraint(x0,g_ee_x_str,x0,'initial', xVars) + ... % x(0)
    constraint(y0,g_ee_y_str,y0,'initial', xVars) + ... % y(0)
    constraint(xf,g_ee_x_str,xf,'final', xVars) + ...     % Final position, time is normalised
    constraint(yf,g_ee_y_str,yf,'final', xVars);

% Define Cost Function
% ====================
Cost = cost('a1d^2+a2d^2','trajectory'); % Minimise energy

% Collocation Points, using Gaussian Quadrature formula
% =====================================================

breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';

HL = [0 colpnts hl];
HL = linspace(0,hl,20);


% Path where the problem related files will be stored
% ===================================================
pathName = './';  % Save it all in the current directory.

% Name of the problem, will be used to identify files
% ===================================================
probName = 'planar2r';

% List of trajectories used in the problem
% ========================================
TrajList = traj.trajList(a1,a1d,a2,a2d);

nlp = ocp2nlp(TrajList, Cost,Constr, HL, ParamList,pathName,probName);
snset('Minimize');



xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0,1,100);
a1_val = linspace(0,pi/2,100);
a2_val = linspace(0,0,100);
a1_sp = createGuess(a1,Time,a1_val);
a2_sp = createGuess(a2,Time,a2_val);
init = [a1_sp.coefs a2_sp.coefs]';% + 0.001*rand(nlp.nIC,1);
%init = zeros(nlp.nIC,1);

ghSnopt = snoptFunction(nlp);
tic;
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], ghSnopt);
toc;
F(1)

sp = getTrajSplines(nlp,x);
a1SP = sp{1};
a2SP = sp{2};


refinedTimeGrid = linspace(min(HL),max(HL),100);

A1 = fnval(a1SP,refinedTimeGrid);
A1d = fnval(fnder(a1SP),refinedTimeGrid);

A2 = fnval(a2SP,refinedTimeGrid);
A2d = fnval(fnder(a2SP),refinedTimeGrid);

% Planar 2-R Arm joint trajectory
figure(1);
plot(A1,A2,'b');
xlabel('Joint 1 (rad)'); ylabel('Joint 2 (rad)');
title('Joint Trajectory');

% Planar 2-R Arm animation
figure(2);
planar2R_l1_plot_hdl = plot(0, 0, 'r', 'Linewidth', 2);
hold on;
planar2R_l2_plot_hdl = plot(0, 0, 'm', 'Linewidth', 2);
planar2R_traj_plot_hdl = plot(0, 0, 'c--', 'Linewidth', 1);

plot(0, 0, 'bo');       % joint 1
planar2R_j2_plot_hdl = plot(0, 0, 'bo');       % joint 2
planar2R_ee_plot_hdl = plot(0, 0, 'bo');       % e-e

plot(x0, y0, 'gd');     % e-e start
plot(xf, yf, 'rd');     % e-e end

total_len = l_1 + l_2;
xlim([-total_len, total_len]*1.1); ylim([-total_len, total_len]*1.1);
xlabel('X-Axis (Spatial)'); ylabel('Y-Axis (Spatial)'); title('Planar 2-R Arm');
hold off;

planar2r_traj = [];
for ii = 1:length(A1)
  set(planar2R_l1_plot_hdl, 'XData', [0, l_1*cos(A1(ii))], 'YData', [0, l_1*sin(A1(ii))]);
  set(planar2R_l2_plot_hdl, 'XData', [l_1*cos(A1(ii)), l_1*cos(A1(ii))+l_2*cos(A1(ii)+A2(ii))], ...
                            'YData', [l_1*sin(A1(ii)), l_1*sin(A1(ii))+l_2*sin(A1(ii)+A2(ii))]);
                          
  set(planar2R_j2_plot_hdl, 'XData', l_1*cos(A1(ii)), 'YData', l_1*sin(A1(ii)));
  set(planar2R_ee_plot_hdl, 'XData', l_1*cos(A1(ii))+l_2*cos(A1(ii)+A2(ii)), ...
                            'YData', l_1*sin(A1(ii))+l_2*sin(A1(ii)+A2(ii)));

  planar2r_traj = [planar2r_traj, ...
            [l_1*cos(A1(ii))+l_2*cos(A1(ii)+A2(ii)) ; l_1*sin(A1(ii))+l_2*sin(A1(ii)+A2(ii))]];
  set(planar2R_traj_plot_hdl, 'XData', planar2r_traj(1, :), 'YData', planar2r_traj(2, :));
  
  pause(0.1);   % arbitrary length delay (for viewability)
end

