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
SNOPTPATH = '../../../../snopt';
addpath([ SNOPTPATH ]);
addpath([ SNOPTPATH '/matlab/matlab/' ]);
addpath([ SNOPTPATH '/matlab/mex/' ]);

ninterv = 2;
hl = 1.0;

x0 = 0; xf = 1.2;
y0 = 0; yf = 1.6;
eps = 0.0001;


% Create trajectory variables
% ===========================
x = traj('x', ninterv,2,3); % Arguments are ninterv, smoothness, order
y = traj('y', ninterv,2,3);

% Create derivatives of trajectory variables
% ==========================================
xd = deriv(x, 'xd');
yd = deriv(y, 'yd');

ParamList = [];
xVars = {'x'; 'y'; 'xd'; 'yd'};

% Define constraints
% ==================
Constr = constraint(x0,'x',x0,'initial', xVars) + ... % x(0)
    constraint(y0,'y',y0,'initial', xVars) + ... % y(0)
    constraint(xf,'x',xf,'final', xVars) + ...     % x(1) = 1,  Final position, time is normalised
    constraint(yf,'y',yf,'final', xVars) + ...     % y(1) = 1
    constraint(0.1,'(x-0.4)^2 + (y-0.5)^2',Inf,'trajectory', xVars) + ... % Dynamics as a path constraint
    constraint(0.1,'(x-0.8)^2 + (y-1.5)^2',Inf,'trajectory', xVars);
% Define Cost Function
% ====================
Cost = cost('xd^2+yd^2','trajectory'); % Minimise energy

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
probName = 'obstacle_avoidance';

% List of trajectories used in the problem
% ========================================
TrajList = traj.trajList(x,xd,y,yd);

nlp = ocp2nlp(TrajList, Cost,Constr, HL, ParamList,pathName,probName);
snset('Minimize');



xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0,1,100);
xval = linspace(0,xf,100);
yval = linspace(0,yf,100);
xsp = createGuess(x,Time,xval);
ysp = createGuess(y,Time,yval);
init = [xsp.coefs ysp.coefs]';% + 0.001*rand(nlp.nIC,1);
%init = zeros(nlp.nIC,1);

ghSnopt = snoptFunction(nlp);
tic;
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], ghSnopt);
toc;
F(1)

sp = getTrajSplines(nlp,x);
xSP = sp{1};
ySP = sp{2};


refinedTimeGrid = linspace(min(HL),max(HL),100);

X = fnval(xSP,refinedTimeGrid);
Xd = fnval(fnder(xSP),refinedTimeGrid);

Y = fnval(ySP,refinedTimeGrid);
Yd = fnval(fnder(ySP),refinedTimeGrid);

figure(1);
plot(X,Y,'b');hold on;
th = [0:0.1:2*pi+0.1];
x = 0.4 + sqrt(.1)*cos(th);
y = 0.5 + sqrt(.1)*sin(th);
x1 = 0.8 + sqrt(.1)*cos(th);
y1 = 1.5 + sqrt(.1)*sin(th);
plot(x,y,'r',x1,y1,'r');axis equal;title('XY position'); xlabel('x'); ylabel('y');
