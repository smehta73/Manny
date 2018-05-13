addpath('/home/manny/Documents/Manny')
addpath('/home/manny/Documents/Manny/Optragen/src')
addpath('/home/manny/Documents/Manny/Optragen')
% MATLIBS = {'control/snopt','control/snopt/matlab/matlab', ...
%             'control/optimal/Optragen/src', 'control/optimal/Optragen/', ...
%             'control/optimal/'};
% ivalab.loadLibraries(MATLIBS);
addpath('autogen');
close all
clear all;
load('withRespectToLeftNew.mat')
load('IPPart2Alphas.mat')
load('IPPart1Alphas.mat')
xRightFootBaseEquation = char(positionOfRightFootBase(1));
yRightFootBaseEquation = char(positionOfRightFootBase(2));
zRightFootBaseEquation = char(positionOfRightFootBase(3));
xRightFrontFootEquation = char(rightFootFront(1));
yRightFrontFootEquation = char(rightFootFront(2));
zRightFrontFootEquation = char(rightFootFront(3));
xCoMwrtLeftFootEquation = char(xCoMwrtLeft);
yCoMwrtLeftFootEquation = char(yCoMwrtLeft);
alphaSymbols = {'a1';'a3';'a4';'a5';'a6';'a12';'a14';'a15';'a16';'a17'};
global nlp;
ninterv = 2;
hl = 1.0;
a1 = traj('a1',ninterv,2,5); 
a3 = traj('a3',ninterv,2,5); 
a4 = traj('a4',ninterv,2,5);
a5 = traj('a5',ninterv,2,5);
a6 = traj('a6',ninterv,2,5); 
a12 = traj('a12',ninterv,2,5); 
a14 = traj('a14',ninterv,2,5); 
a15 = traj('a15',ninterv,2,5); 
a16 = traj('a16',ninterv,2,5); 
a17 = traj('a17',ninterv,2,5); 
a1d = a1.deriv('a1d');
a3d = a3.deriv('a3d');
a4d = a4.deriv('a4d');
a5d = a5.deriv('a5d');
a6d = a6.deriv('a6d');
a12d = a12.deriv('a12d');
a14d = a14.deriv('a14d');
a15d = a15.deriv('a15d');
a16d = a16.deriv('a16d');
a17d = a17.deriv('a17d');
ParamList = {};
startingValues = IPPart2Alphas(:,end);
finalValues = IPPart1Alphas(:,1);
a1InitialConstraint = constraint(startingValues(1),'a1',startingValues(1),'initial','a1');
a3InitialConstraint = constraint(startingValues(2),'a3',startingValues(2),'initial','a3');
a4InitialConstraint = constraint(startingValues(3),'a4',startingValues(3),'initial','a4');
a5InitialConstraint = constraint(startingValues(4),'a5',startingValues(4),'initial','a5');
a6InitialConstraint = constraint(startingValues(5),'a6',startingValues(5),'initial','a6');
a12InitialConstraint = constraint(startingValues(6),'a12',startingValues(6),'initial','a12');
a14InitialConstraint = constraint(startingValues(7),'a14',startingValues(7),'initial','a14');
a15InitialConstraint = constraint(startingValues(8),'a15',startingValues(8),'initial','a15');
a16InitialConstraint = constraint(startingValues(9),'a16',startingValues(9),'initial','a16');
a17InitialConstraint = constraint(startingValues(10),'a17',startingValues(10),'initial','a17');
initialConstraints = a1InitialConstraint + a3InitialConstraint + a4InitialConstraint + a5InitialConstraint + a6InitialConstraint + a12InitialConstraint + a14InitialConstraint + a15InitialConstraint + a16InitialConstraint + a17InitialConstraint;
a1FinalConstraint = constraint(-finalValues(6),'a1',-finalValues(6),'final','a1');
a3FinalConstraint = constraint(finalValues(7),'a3',finalValues(7),'final','a3');
a4FinalConstraint = constraint(finalValues(8),'a4',finalValues(8),'final','a4');
a5FinalConstraint = constraint(finalValues(9),'a5',finalValues(9),'final','a5');
a6FinalConstraint = constraint(-finalValues(10),'a6',-finalValues(10),'final','a6');
a12FinalConstraint = constraint(-finalValues(1),'a12',-finalValues(1),'final','a12');
a14FinalConstraint = constraint(finalValues(2),'a14',finalValues(2),'final','a14');
a15FinalConstraint = constraint(finalValues(3),'a15',finalValues(3),'final','a15');
a16FinalConstraint = constraint(finalValues(4),'a16',finalValues(4),'final','a16');
a17FinalConstraint = constraint(-finalValues(5),'a17',-finalValues(5),'final','a17');
finalConstraints = a1FinalConstraint + a3FinalConstraint + a4FinalConstraint + a5FinalConstraint + a6FinalConstraint + a12FinalConstraint + a14FinalConstraint + a15FinalConstraint + a16FinalConstraint + a17FinalConstraint;
zRightFootFrontTrajectoryConstraint = constraint(0,zRightFrontFootEquation, inf, 'trajectory', alphaSymbols);
allConstraints = initialConstraints + finalConstraints + zRightFootFrontTrajectoryConstraint;
Cost = cost('a1d^2+a3d^2+a4d^2+a5d^2+a6d^2+a12d^2+a14d^2+a15d^2+a16d^2+a17d^2','trajectory');
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';
HL = linspace(0,hl,20);
pathName = './';
probName = 'planarMotionLeg';
TrajList = traj.trajList(a1,a1d,a3,a3d,a4,a4d,a5,a5d,a6,a6d,a12,a12d,a14,a14d,a15,a15d,a16,a16d,a17,a17d);
nlp = ocp2nlp(TrajList,Cost,allConstraints,HL,ParamList,pathName,probName);
xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);
Time = linspace(0,1,100);
a1val = linspace(0,1,100);
a3val = linspace(0,1,100);
a4val = linspace(0,1,100);
a5val = linspace(0,1,100);
a6val = linspace(0,1,100);
a12val = linspace(0,1,100);
a14val = linspace(0,1,100);
a15val = linspace(0,1,100);
a16val = linspace(0,1,100);
a17val = linspace(0,1,100);
a1sp = createGuess(a1,Time,a1val);
a3sp = createGuess(a3,Time,a3val);
a4sp = createGuess(a4,Time,a4val);
a5sp = createGuess(a5,Time,a5val);
a6sp = createGuess(a6,Time,a6val);
a12sp = createGuess(a12,Time,a12val);
a14sp = createGuess(a14,Time,a14val);
a15sp = createGuess(a15,Time,a15val);
a16sp = createGuess(a16,Time,a16val);
a17sp = createGuess(a17,Time,a17val);
init = [a1sp.coefs a3sp.coefs a4sp.coefs a5sp.coefs a6sp.coefs a12sp.coefs a14sp.coefs a15sp.coefs a16sp.coefs a17sp.coefs];
tic;
ghSnopt = ipoptFunction(nlp);
toc;
[~,~,nobj, nlinConstr, nnlConstr]=ghSnopt(init');
nFreeVar=length(init);
nConstraint = nlinConstr+nnlConstr;
profile on;
tic;
[x, info]=optragen_ipopt(ghSnopt,nobj,nFreeVar,nConstraint,init,xlow,xupp,[nlp.LinCon.lb;nlp.nlb],[nlp.LinCon.ub;nlp.nub]);
toc;
sp = getTrajSplines(nlp,x);
a1SP = sp{1};
a3SP = sp{2};
a4SP = sp{3};
a5SP = sp{4};
a6SP = sp{5};
a12SP = sp{6};
a14SP = sp{7};
a15SP = sp{8};
a16SP = sp{9};
a17SP = sp{10};
refinedTimeGrid = linspace(min(HL),max(HL),100);
a1 = fnval(a1SP,refinedTimeGrid);
a3 = fnval(a3SP,refinedTimeGrid);
a4 = fnval(a4SP,refinedTimeGrid);
a5 = fnval(a5SP,refinedTimeGrid);
a6 = fnval(a6SP,refinedTimeGrid);
a12 = fnval(a12SP,refinedTimeGrid);
a14 = fnval(a14SP,refinedTimeGrid);
a15 = fnval(a15SP,refinedTimeGrid);
a16 = fnval(a16SP,refinedTimeGrid);
a17 = fnval(a17SP,refinedTimeGrid);
t=1;
manny = Manny(zeros(1,22));
xCOMS = zeros(1,100);
yCOMS = zeros(1,100);
zRightFootBasePosition = zeros(1,100);
yRightFootBasePosition = zeros(1,100);
xRightFootBasePosition = zeros(1,100);
IPPart3Alphas = [a1;a3;a4;a5;a6;a12;a14;a15;a16;a17];
save('IPPart3Alphas.mat','IPPart3Alphas')
tempAlphas = zeros(1,22);
for i = 1:100
 tempAlphas(1) = a1(i);
 tempAlphas(3) = a3(i);
 tempAlphas(4) = a4(i);
 tempAlphas(5) = a5(i);
 tempAlphas(6) = a6(i);
 tempAlphas(12) = a12(i);
 tempAlphas(14) = a14(i);
 tempAlphas(15) = a15(i);
 tempAlphas(16) = a16(i);
 tempAlphas(17) = a17(i);
 manny = manny.setGTransforms(tempAlphas);
 [xCoM,yCoM] = manny.CoM(1);
 xCOMS(i) = xCoM;
 yCOMS(i) = yCoM;
  [rightFootBase,~,~,~,rightFootFront] = footPositions(manny);
 zRightFootBasePosition(i) = rightFootBase(3);
 yRightFootBasePosition(i) = rightFootBase(2);
 xRightFootBasePosition(i) = rightFootBase(1); 
 zRightFootFrontPosition(i) = rightFootFront(3);
end
figure
plot(xCOMS);
hold on
plot(yCOMS);
title('Center of Mass')
legend('Center of Mass:x','Center of Mass:y')
figure
plot(zRightFootFrontPosition)
hold on
plot(zRightFootBasePosition,'m')
hold on
plot(xRightFootBasePosition,'c')
hold on
plot(yRightFootBasePosition,'b')
hold on
title('Foot Position Base')
legend('z right foot front position','z right foot base position ','x right foot base position','y right foot base postion')
figure
plot(a1,'k')
hold on
plot(a3,'c')
hold on
plot(a4,'m')
hold on
plot(a5,'r')
hold on
plot(a6,'g')
legend('a1(hip-x)','a3(hip-y)','a4(knee-y)','a5(foot-y)','a6(foot-x)')
title('Left Foot Angles')
figure
plot(a12,'k')
hold on
plot(a14,'c')
hold on
plot(a15,'m')
hold on
plot(a16,'r')
hold on
plot(a17,'g')
legend('a12(hip-x)','a14(hip-y)','a15(knee-y)','a16(foot-y)','a17(foot-x)')
title('Right Foot Angles')
figure
alphas = zeros(1,22);
manny = Manny(alphas);
visualInd = [1:5:96 100];
for index = 1:length(visualInd)
index = visualInd(index);
alphas(1) = a1(index);
alphas(3) = a3(index);
alphas(4) = a4(index);
alphas(5) = a5(index);
alphas(6) = a6(index);
alphas(12) = a12(index);
alphas(14) = a14(index);
alphas(15) = a15(index);
alphas(16) = a16(index);
alphas(17) = a17(index);
manny = manny.setGTransforms(alphas);
visualization(manny,1)
view(0,0)
pause(.05)
end