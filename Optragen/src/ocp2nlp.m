% =======================================================================
%   OPTRAGEN
%   Copyright (c) 2005 by
%   Raktim Bhattacharya, (raktim@aero.tamu.edu)
%   Department of Aerospace Engineering
%   Texas A&M University.
%   All right reserved.
% =======================================================================

function [nlp] = ocp2nlp(TrajList, Cost,Constr,HL, ParamList,pathName,probName)

%ban1 = sprintf('\t\t\t\t OPTRAGEN ver 2.0\n\t\t\t\t Copyright (c) by Raktim Bhattacharya (raktim@aero.tamu.edu)');
%ban2 = sprintf('\t\t\t\t Department of Aerospace Engineering\n\t\t\t\t Texas A&M University. \n\t\t\t\t All rights reserved');
%ban3 = sprintf('\t\t\t\t ====================================');
%ban = sprintf('%s\n%s\n%s\n\n',ban1,ban2,ban3);
%
%disp(ban);

% Get Optimisation parameters
% ============================
OutputInfo = getOutputInfo(TrajList);
OutputSequence = createOutputSequence(OutputInfo);
nlp = getOptParam(OutputInfo, Cost, Constr, HL, ParamList);

ColMat = getCollocMatrix(nlp);

% Get Linear Constraints
% ======================
LinCon = getNlpLinearA(nlp,ColMat);


% Get Nonlinear Constraint Bounds
% ===============================
nlb = [];
nub = [];
if nlp.nnlic ~= 0
  nlb = [nlb;nlp.nlic_lb'];
  nub = [nub;nlp.nlic_ub'];
end

if nlp.nnltc ~= 0
  v1 = repmat(nlp.nltc_lb,nlp.nbps,1);
  v2 = repmat(nlp.nltc_ub,nlp.nbps,1);
  nlb = [nlb;v1(:)];
  nub = [nub;v2(:)];
end

if nlp.nnlfc ~= 0
  nlb = [nlb;nlp.nlfc_lb'];
  nub = [nub;nlp.nlfc_ub'];
end


% Create Cost Functions
% =====================
icfCode = genSource.createFunction(OutputSequence,Cost,'initial', ...
                                                  probName, ParamList);
tcfCode = genSource.createFunction(OutputSequence,Cost,'trajectory', ...
                                                  probName, ParamList);
fcfCode = genSource.createFunction(OutputSequence,Cost,'final', ...
                                                  probName, ParamList);

% Create Constraint Functions
% ===========================
nlicfCode = genSource.createFunction(OutputSequence,Constr,'initial', ...
                                                    probName,ParamList);
nltcfCode = genSource.createFunction(OutputSequence,Constr,'trajectory', ...
                                                    probName,ParamList);
nlfcfCode = genSource.createFunction(OutputSequence,Constr,'final', ...
                                                    probName,ParamList);
nlgcfCode = genSource.createFunction(OutputSequence,Constr,'galerkin', ...
                                                    probName,ParamList);


% Write functions for costs
% =============================
fPtr = genSource.createHeader(probName, pathName);
genSource.writeFunction(fPtr, 'icf', icfCode);
genSource.writeFunction(fPtr, 'tcf', tcfCode);
genSource.writeFunction(fPtr, 'fcf', fcfCode);

% Write functions to nonlinear constraints
% =============================================
genSource.writeFunction(fPtr, 'nlicf', nlicfCode);
genSource.writeFunction(fPtr, 'nltcf', nltcfCode);
genSource.writeFunction(fPtr, 'nlfcf', nlfcfCode);
genSource.writeFunction(fPtr, 'nlgcf', nlgcfCode);

genSource.createFooter(fPtr);

% Create nlp data structure
% =========================
nlp.OutputSequence = OutputSequence;

for i=1:nlp.nbps
  M = [];
  for j=1:nlp.nout
    ncoef = size(ColMat(j).Data,3);
    tmpMat = reshape(ColMat(j).Data(:,i,:),nlp.maxderiv(j),ncoef);
    M = diagonalise(M,tmpMat);
  end
  B(:,:,i) = M;
end

nlp.B = B;
nlp.probName = probName;
nlp.probFuncs = feval(probName);
nlp.LinCon = LinCon;

% Get basis functions for Galerkin Projection
% ===========================================
if nlp.nnlgc ~= 0
  nlp.BasisFcn = getBspBasisFcn(nlp.ninterv,nlp.order,nlp.smoothness,HL);
  nBasis = size(nlp.BasisFcn,2);
  nlb = [nlb;zeros(nBasis*nlp.nnlgc,1)];
  nub = [nub;zeros(nBasis*nlp.nnlgc,1)];
end
nlp.nlb = nlb;
nlp.nub = nub;

