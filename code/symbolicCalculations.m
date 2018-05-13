syms  a1 a3 a4 a5 a6 a12 a14 a15 a16 a17 
alphasSymbolic = [a1 0 a3 a4 a5 a6 0 0 0 0 0  a12 0 a14 a15 a16 a17 0 0 0 0 0];
MannySymbolic = Manny(alphasSymbolic);
[xCoMwrtLeft,yCoMwrtLeft] = MannySymbolic.CoM(1);
[positionOfRightFootBase,~,~,~,rightFootFront] = MannySymbolic.footPositions;
% noDragging1 = (positionOfLeftFootBase(1) + 1.4)^2 + (positionOfLeftFootBase(3) + .2)^2;