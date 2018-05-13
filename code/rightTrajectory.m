close all
load('IPPart2Alphas.mat')
load('IPPart1Alphas.mat')
load('IPPart3Alphas.mat')
addpath('/Users/siddharthmehta/Documents/Georgia Tech/Research/Manny-master')
all = [IPPart1Alphas IPPart2Alphas IPPart3Alphas];
a1 = -all(1+5,:);
a3 = all(2+5,:);
a4 = all(3+5,:);
a5 = all(4+5,:);
a6 = -all(5+5,:);
a12 = -all(6-5,:);
a14 = all(7-5,:);
a15 = all(8-5,:);
a16 = all(9-5,:);
a17 = -all(10-5,:);
manny = Manny(zeros(1,22));
tempAlphas = zeros(1,22);
for i = 1:300
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
 [xCoM,yCoM] = manny.CoM(2);
 xCOMS(i) = xCoM;
 yCOMS(i) = yCoM;
 [~,leftFootBase] = footPositions(manny);
 zLeftFootBasePosition(i) = leftFootBase(3);
 yLeftFootBasePosition(i) = leftFootBase(2);
 xLeftFootBasePosition(i) = leftFootBase(1); 
end
figure
plot(xCOMS);
hold on
plot(yCOMS);
title('Center of Mass')
legend('Center of Mass:x','Center of Mass:y')
figure
plot(zLeftFootBasePosition,'m')
hold on
plot(xLeftFootBasePosition,'c')
hold on
plot(yLeftFootBasePosition,'b')
title('Foot Position Base')
legend('z left foot base position ','x left foot base position','y left foot base postion')
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
visualInd = [1:10:291 300];
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
visualization(manny,2)
view(180,0)
pause(.05)
end