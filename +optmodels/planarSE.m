%================================ planarSE ===============================
%
% @class    planarSE
%
% @brief    Interface abstraction for trajectory generation of planar objects.
%
%  
%  Attempts to create a simpler interface for the description and
%  construction of optimal control trajectory generation problems.
%  Requires optragen as the underlying base.
%
%================================ planarSE ===============================

%
% @file     planarSE.m
%
% @author   Patricio A. Vela,   pvela@gatech.edu
% @date     2016/08/10 [created]
%
% @note
%   set indent to 2 spaces.
%   set tab to 4 spaces, with conversion.
%
%================================ planarSE ===============================
classdef planarSE < handle


properties
  time = 't';       %! Symbolic scalar variable for time.
  x;                %! Symbolic vector describing state.
  u;                %! Symbolic vector describing (control) input.
  xprime;           %! Symbolic vector describing state velocity.

  xTraj;            %! Vector of state trajectory approximation functions.
  uSignal;          %! Vector of input signal approximation functions.

  constraints;      %! Set of constraints to apply.
  ic;               %! Initial conditions.
  fc;               %! Final conditions.
  timec;            %! Time constraints.
  obsc;             %! Obstacle constraints.
  dync;             %! Dynamics.
  inpc;             %! Input constraints.
end


%)
%
%============================ Member Functions ===========================
%
%(

methods


  %============================== planarSE =============================
  %
  % @brief      Constructor for planar trajectory generation solver.
  %
  % @param[in]  xSym    String describing symbolic version of state vector.
  % @param[in]  uSym    String describing symbolic version of control input.
  % @param[in]  tSym    String describing symbolic version of time [optional].
  %
  function this = planarSE(xSym, uSym, tSym)

  this.x = xSym;
  this.u = uSym;

  if (nargin > 2)
    this.time = tSym;
  end

  end

  %===== setState =====
  %
  % @brief      Define the state variables to use during the optimization.
  %
  % @param[in]  xVars   The curve approximation variables to use.
  %

  %============================ initialState ===========================
  %
  % @brief      Define the initial state of the trajectory.
  %
  % @param[in]  xI      The desired initial state.
  %
  function initialState(this, xI)

  this.ic = constraint(xI(1), x(1), xI(1), 'initial');

  for ii=1:length(x)
    this.ic = this.ic + constraint(xI(ii), x(ii), xI(ii), 'initial');
  end

  end

  %
  %------------------- Interface: Problem Description ------------------
  %
  %(

  %============================= finalState ============================
  %
  % @brief      Define the final state of the trajectory.
  %
  % @param[in]  xF      The desired final state.
  %
  function finalState(this, xF)

  this.ic = constraint(xF(1), x(1), xF(1), 'final');

  for ii=1:length(x)
    this.ic = this.ic + constraint(xF(ii), x(ii), xF(ii), 'final');
  end
  
  end

  %============================= finalTime =============================
  %
  % @brief      Define the final time point of the trajectory.
  %
  % @param[in]  xF      The desired time point.
  %
  function finalTime(this, tF)

  this.tc = constraint(tF(1), this.time, tF(1), 'final');

  end


  %============================ addObstacle ============================
  %
  % @brief      Add the obstacle to the set of constraints.
  %
  % @param[in]  theObs  An instance of a obstacle class.
  %
  function addObstacle(this, theObs)

  % Use obstacle class to define constraints.
  if (isempty(this.constraints))
    this.constraints = constraint( BLAH, BLAH );
  else
    this.constraints = this.constraints ...
                        + constraint( BLAH, BLAH );
  end

  end

  %============================== dynamics =============================
  %
  % @brief      Define the dynamics to follow.
  %
  % @param[in]  X       The equations of motion vector field (sybolic).
  % @param[in]  lims    Tolerance in fitting of dynamics [optional].
  %
  function dynamics(this, X, lims)

  if ( (nargin < 3) || isempty(lims) )
    lims = zeros(size(X,1), 2);
  end

  this.dync = constraint(lims(1), X(1), lims(1), 'trajectory');

  for ii=1:length(x)
    this.dync = this.dync ...
                  + constraint(-lims(ii), X(ii), lims(ii), 'trajectory');
  end

  end

  %=========================== limitControls ===========================
  %
  % @brief      Define lower/uppoer limits on control inputs.
  %
  % @param[in]  uLims   Limits of control inputs (lower/upper, rowwise).
  %
  function limitControls(this, uLims)

  assert(size(this.u,2) == size(uLims, 2));

  this.inpc = constraint(uLims(1,1), this.u(1), uLims(1,2), 'trajectory');

  for ii=1:size(this.u,2)
    this.inpc = this.inpc ...
              + constraint(uLims(ii,1), this.u(ii), uLims(ii,2), 'trajectory');
  end

  end


  %=============================== cost ================================
  %
  % @brief      Define the cost functional to optimize over.
  %
  %
  function cost(this, costStr)

  this.cost = cost(costStr, 'trajectory');

  end

  %========================= collocationPoints =========================
  %
  % @brief      Define the collocation points to use.
  function collocationPoints(this, cpts)

  end

  %)
  %
  %---------- Interface: Construction Optimal Control Problem ----------
  %
  %(

  %=============================== build ===============================
  %
  % @brief      Build the optimal control problem from its description.
  %
  function build(this)

  %do some work here.

  end

  %============================== rebuild ==============================
  %
  % @brief      Rebuild model assuming minor changes.
  %
  function rebuild(this)

  end

  %=============================== solve ===============================
  %
  % @brief      Solve the defined optimal control problem.
  %
  function solve(this)

  end


  %TODO: What happens when only part changes?
  %TODO: Figure out how to change trivial parts and update quickly.


  %TODO: What other functions are of use/importance?

  %)

end

%)
%
%======================== Static Class Functions =======================
%
%(

methods(Static)


  %========================== initialCondition =========================
  %
  % @brief  Specify initial condition constraints.
  %
  % @param[in]  gSym    Symbolic string of SE(2) variable.
  % @param[in]  gInit   Numerical value of initial condition.
  %
  function ic = initialCondition(gSym, gInit)

  ic = constraint.initialCondition(gSym{1}, gInit(1))       ... % x(0)
         + constraint.initialCondition(gSym{2}, gInit(2))   ... % y(0)
         + constraint.initialCondition(gSym{3}, gInit(3));  ... % theta(0)

  end

  %=========================== finalCondition ==========================
  %
  % @brief  Specify final condition constraints.
  %
  % @param[in]  gSym    Symbolic string of SE(2) variable.
  % @param[in]  gFinal  Numerical value of final condition.
  %
  function ic = finalCondition(gSym, gfinal)

  ic = constraint.finalCondition(gSym{1}, gfinal(1))       ... % x(0)
         + constraint.finalCondition(gSym{2}, gfinal(2))   ... % y(0)
         + constraint.finalCondition(gSym{3}, gfinal(3));  ... % theta(0)

  end

  %=========================== finalPosition ===========================
  %
  % @brief  Specify final position of the SE(2) state (orientation
  %         independent).
  %
  % @param[in]  gSym    Symbolic string of SE(2) variable.
  % @param[in]  pfinal  Numerical value of final position.
  %
  function fc = finalPosition(gSym, pFinal)

  fc = constraint.finalCondition(gSym{1}, pFinal(1))       ... % x(0)
         + constraint.finalCondition(gSym{2}, pFinal(2));  ... % y(0)

  end

  %=========================== dynamicsHilare ==========================
  %
  % @brief  Specify se(2) constraints consistent with hilare robot.
  %
  % The simplest Hilare robot model controls forward velocity and also
  % angular rate.  Since the angular rate is somewhat decoupled from the
  % translational component, it is managed individually. However, the
  % planar position has non-trivial dynamics, so this function sets up
  % those constraints.
  %
  % @param[in]  gpSym   Symbolic string of SE(2) derivative.
  % @param[in]  gSym    Symbolic string of SE(2) variable.
  % @param[in]  vSym    Symbolic string of input variable.
  % @param[in]  vEff    Control gain factor on speed control input.
  %
  function dc = dynamicsHilare(gpSym, gSym, vSym, vEff)

  dynEps = 1e-7;

  if (~isstr(vEff))
    vEff = num2str(vEff(1));
  end

  dStr{1} = [ gpSym{1} '-' vEff '*' vSym '*cos(' gSym{3} ')' ];
  dStr{2} = [ gpSym{2} '-' vEff '*' vSym '*sin(' gSym{3} ')' ];

  xSym = cat(1, gSym, {gpSym{1}; gpSym{2}}, vSym);
  dc = constraint(-dynEps, dStr{1}, dynEps, 'trajectory', xSym) ...
       + constraint(-dynEps, dStr{2}, dynEps, 'trajectory', xSym);

  end

  %============================== obsCircle ==============================
  %
  % @brief Add a circular obstacle constraint with a presumed point mass
  %         robot.
  %
  function oc = obsCircle(gSym, xCent, rad)

  oc = constraint.obsCircle({gSym{1}; gSym{2}}, xCent, rad);

  end

  function oc = obsSquareApprox(gSym, xCent, rad, order)

  oc = constraint.obsSquareApprox({gSym{1}; gSym{2}}, xCent, rad, order);

  end

  function bc = squareBodyAvoidPoint(gSym, pWorld, scale, power)

  if isscalar(scale)
      scalingV= [scale scale];
  else
      scalingV= [scale(1) scale(2)];
          
  end
  cosStr = ['cos(', gSym{3}, ')'];
  sinStr = ['sin(', gSym{3}, ')'];
  xradStr = ['(', num2str(scalingV(1)), ')'];
  yradStr = ['(', num2str(scalingV(2)), ')'];
  powStr = ['(', num2str(power), ')'];
  pWorldStr1 = ['(',num2str(pWorld(1)),')'];
  pWorldStr2 = ['(',num2str(pWorld(2)),')'];
  diffStr{1} = [ '(',pWorldStr1 , '-', gSym{1}, ')' ];
  diffStr{2} = [ '(', pWorldStr2, '-', gSym{2} , ')' ];

  nLpStr = ['(( (',diffStr{1},'*',cosStr,'+',diffStr{2},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr = [nLpStr , ' + ' , ...
             [ '( (-',diffStr{1},'*',sinStr,'+',diffStr{2},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];

  nLpStr
  bc = constraint(1, nLpStr, Inf, 'trajectory',gSym);

  end
%========================== circleBodyAvoidRect ==========================
  %
  % @brief  Create a constraint for rectangular obstacle with a circular
  % body robot. 6 constraints (2 Rec + 4 circle)
  %
  % @param[in]  gSym    Symbolic description of state vector.
  % @param[in]  obs Symbolic description of obstacle shape and orientation.
  % @param[in]  scale   Ratio between the edges of obstacle (scalar or 2-vector).
  % @param[in]  power   Order of Lp constraint approximating the sqyare.
  % @param[in]  rad    Radius of the robot.
  %
  function bc = circleBodyAvoidRect(gSym, obs, scale, power, rad)

  if isscalar(scale)
      scalingV= [scale scale];
  else
      scalingV= [scale(1) scale(2)];
          
  end
  cosStr = ['cos(', num2str(obs(3)), ')'];
  sinStr = ['sin(', num2str(obs(3)), ')'];
  xradStr = ['(', num2str(scalingV(1)), ')'];
  xradStrExt =['(', num2str(scalingV(1)+rad), ')'];
  yradStr = ['(', num2str(scalingV(2)), ')'];
  yradStrExt = ['(', num2str(scalingV(2)+rad), ')'];
  radStr =['(', num2str(rad),')'];
  
  powStr = ['(', num2str(power), ')'];
  pWorldStr1 = ['(',num2str(obs(1)),')'];
  pWorldStr2 = ['(',num2str(obs(2)),')'];
  diffStr{1} = [ '(',gSym{1},'-',pWorldStr1, ')' ];
  diffStr{2} = [ '(',gSym{2},'-',pWorldStr2, ')' ];
  diffRotStr{1} =['(',diffStr{1},'*',cosStr,'+',diffStr{2},'*',sinStr,')'];
  diffRotStr{2} =['(-',diffStr{1},'*',sinStr,'+',diffStr{2},'*',cosStr,')'];
  
  diffRotStr{3} = [ '(',diffRotStr{1},'-(', xradStr, ') )' ]; % NE corner - x cord
  diffRotStr{4} = [ '(',diffRotStr{2},'-(', yradStr, ') )' ]; % NE corner - y cord
  diffRotStr{5} = [ '(',diffRotStr{1},'-(', xradStr, ') )' ]; % SE corner - x cord
  diffRotStr{6} = [ '(',diffRotStr{2},'-(-', yradStr, ') )' ]; % SE corner - y cord
  diffRotStr{7} = [ '(',diffRotStr{1},'-(-', xradStr, ') )' ]; % SW corner - x cord
  diffRotStr{8} = [ '(',diffRotStr{2},'-(-', yradStr, ') )' ]; % SW corner - y cord
  diffRotStr{9} = [ '(',diffRotStr{1},'-(-', xradStr, ') )' ]; % NW corner - x cord
  diffRotStr{10} = [ '(',diffRotStr{2},'-(', yradStr, ') )' ]; % NW corner - y cord
  
  nLpStr1 = ['((',diffRotStr{1}, '/', xradStrExt, ' )^', powStr ];
  nLpStr1 = [nLpStr1 , ' + (',diffRotStr{2}, '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ];
  nLpStr2 = ['((',diffRotStr{1}, '/', xradStr, ' )^', powStr ];
  nLpStr2 = [nLpStr2 , ' + (',diffRotStr{2}, '/',yradStrExt,' )^',powStr, ')^(1/',powStr,')' ];
  nLpStr3 = ['((',diffRotStr{3},')^2 + (',diffRotStr{4},')^2)/(',radStr, '^2)']; % NE
  nLpStr4 = ['((',diffRotStr{5},')^2 + (',diffRotStr{6},')^2)/(',radStr, '^2)']; % SE
  nLpStr5 = ['((',diffRotStr{7},')^2 + (',diffRotStr{8},')^2)/(',radStr, '^2)']; % SW
  nLpStr6 = ['((',diffRotStr{9},')^2 + (',diffRotStr{10},')^2)/(',radStr, '^2)']; % NW
  
  
  nLpStr1
  nLpStr2
  nLpStr3
  nLpStr4
  nLpStr5
  nLpStr6
  
  bc = constraint(1, nLpStr1, Inf, 'trajectory',gSym)...   % X extended rectangle
         + constraint(1,nLpStr2,Inf, 'trajectory',gSym)...      % Y extended rectangle
         + constraint(1,nLpStr3,Inf, 'trajectory',gSym)...      % NE extended rectangle
         + constraint(1,nLpStr4,Inf, 'trajectory',gSym)...      % SE extended rectangle
         + constraint(1,nLpStr5,Inf, 'trajectory',gSym)...      % SW extended rectangle
         + constraint(1,nLpStr6,Inf, 'trajectory',gSym);     % NW extended rectangle
        
  end
  
  %========================== circleBodyAvoidRectSimp ==========================
  %
  % @brief  Create a constraint for rectangular obstacle with a circular
  % body robot. Simple version: 1 constraints extended by the radius of the
  % robot.
  %
  % @param[in]  gSym    Symbolic description of state vector.
  % @param[in]  obs Symbolic description of obstacle shape and orientation.
  % @param[in]  scale   Ratio between the edges of obstacle (scalar or 2-vector).
  % @param[in]  power   Order of Lp constraint approximating the sqyare.
  % @param[in]  rad    Radius of the robot.
  %
  function bc = circleBodyAvoidRectSimp(gSym, obs, scale, power, rad)

  if isscalar(scale)
      scalingV= [scale scale];
  else
      scalingV= [scale(1) scale(2)];
          
  end
  cosStr = ['cos(', num2str(obs(3)), ')'];
  sinStr = ['sin(', num2str(obs(3)), ')'];
  xradStrExt =['(', num2str(scalingV(1)+rad), ')'];
  yradStrExt = ['(', num2str(scalingV(2)+rad), ')'];
  
  powStr = ['(', num2str(power), ')'];
  pWorldStr1 = ['(',num2str(obs(1)),')'];
  pWorldStr2 = ['(',num2str(obs(2)),')'];
  diffStr{1} = [ '(',gSym{1},'-',pWorldStr1, ')' ];
  diffStr{2} = [ '(',gSym{2},'-',pWorldStr2, ')' ];
  diffRotStr{1} =['(',diffStr{1},'*',cosStr,'+',diffStr{2},'*',sinStr,')'];
  diffRotStr{2} =['(-',diffStr{1},'*',sinStr,'+',diffStr{2},'*',cosStr,')'];
  
  nLpStr1 = ['((',diffRotStr{1}, '/', xradStrExt, ' )^', powStr ];
  nLpStr1 = [nLpStr1 , ' + (',diffRotStr{2}, '/',yradStrExt,' )^',powStr, ')^(1/',powStr,')' ];
  
  nLpStr1
  bc = constraint(1, nLpStr1, Inf, 'trajectory',gSym);   % X extended rectangle
        
  end
  
  %========================== RecBodyAvoidRect ==========================
  %
  % @brief  Create a constraint for rectangular obstacle with a rectangular
  % body robot. 8 constraints (4 Rob corner + 4 Obs corner)
  %
  % @param[in]  gSym    Symbolic description of state vector.
  % @param[in]  obs Symbolic description of obstacle shape and orientation.
  % @param[in]  scale   Ratio between the edges of obstacle (scalar or 2-vector).
  % @param[in]  power   Order of Lp constraint approximating the sqyare.
  % @param[in]  rad    Radius of the robot.
  %
  function bc = RectBodyAvoidRect(gSym, rscale, gAffine, obs, oscale, power)

      
  if isscalar(rscale) % Scaling for robot
      rscalingV= [rscale rscale];
  else
      rscalingV= [rscale(1) rscale(2)];        
  end
  if isscalar(oscale) % Scaling for obstacle
      oscalingV= [oscale pscale];
  else
      oscalingV= [oscale(1) oscale(2)];
          
  end
  if isscalar(power) % Order of Robot and Obstacle
      powerV= [power power];
  else
      powerV= [power(1) power(2)];
          
  end
  %--Four obstacle corners with robot body constratint
  
  affineMag=gAffine(1);
  affineAng=gAffine(2);
  affineOri=gAffine(3);
  affinexStr=[num2str(affineMag), '*cos(', gSym{3}, '+(', num2str(affineAng),'))'];
  affineyStr=[num2str(affineMag), '*sin(', gSym{3}, '+(', num2str(affineAng),'))'];
  cosStr = ['cos(', gSym{3}, '+(', num2str(affineOri),'))'];
  sinStr = ['sin(', gSym{3}, '+(', num2str(affineOri),'))'];
  
  xradStr = ['(', num2str(rscalingV(1)), ')'];
  yradStr = ['(', num2str(rscalingV(2)), ')'];
  powStr = ['(', num2str(powerV(1)), ')'];
  pWorldStr1 = ['(',num2str(obs(1)),')'];
  pWorldStr2 = ['(',num2str(obs(2)),')'];
  
  CorStr{1} = [ '(', xradStr, ')' ]; % NE corner - x cord
  CorStr{2} = [ '(', yradStr, ')' ]; % NE corner - y cord
  CorStr{3} = [ '(', xradStr, ')' ]; % SE corner - x cord
  CorStr{4} = [ '(-', yradStr, ')' ]; % SE corner - y cord
  CorStr{5} = [ '(-', xradStr, ')' ]; % SW corner - x cord
  CorStr{6} = [ '(-', yradStr, ')' ]; % SW corner - y cord
  CorStr{7} = [ '(-', xradStr, ')' ]; % NW corner - x cord
  CorStr{8} = [ '(', yradStr, ')' ]; % NW corner - y cord
  CorOrgStr{1} =['(',CorStr{1},'*',cosStr,'-',CorStr{2},'*',sinStr,')'];
  CorOrgStr{2} =['(',CorStr{1},'*',sinStr,'+',CorStr{2},'*',cosStr,')'];
  CorOrgStr{3} =['(',CorStr{3},'*',cosStr,'-',CorStr{4},'*',sinStr,')'];
  CorOrgStr{4} =['(',CorStr{3},'*',sinStr,'+',CorStr{4},'*',cosStr,')'];
  CorOrgStr{5} =['(',CorStr{5},'*',cosStr,'-',CorStr{6},'*',sinStr,')'];
  CorOrgStr{6} =['(',CorStr{5},'*',sinStr,'+',CorStr{6},'*',cosStr,')'];
  CorOrgStr{7} =['(',CorStr{7},'*',cosStr,'-',CorStr{8},'*',sinStr,')'];
  CorOrgStr{8} =['(',CorStr{7},'*',sinStr,'+',CorStr{8},'*',cosStr,')'];
  
  
  diffStr{1} = [ '(',pWorldStr1,'+',CorOrgStr{1}, '-(', gSym{1},'+',affinexStr, ') )' ];
  diffStr{2} = [ '(',pWorldStr2,'+',CorOrgStr{2}, '-(', gSym{2},'+',affineyStr, ') )' ];
  diffStr{3} = [ '(',pWorldStr1,'+',CorOrgStr{3}, '-(', gSym{1},'+',affinexStr, ') )' ];
  diffStr{4} = [ '(',pWorldStr2,'+',CorOrgStr{4}, '-(', gSym{2},'+',affineyStr, ') )' ];
  diffStr{5} = [ '(',pWorldStr1,'+',CorOrgStr{5}, '-(', gSym{1},'+',affinexStr, ') )' ];
  diffStr{6} = [ '(',pWorldStr2,'+',CorOrgStr{6}, '-(', gSym{2},'+',affineyStr, ') )' ];
  diffStr{7} = [ '(',pWorldStr1,'+',CorOrgStr{7}, '-(', gSym{1},'+',affinexStr, ') )' ];
  diffStr{8} = [ '(',pWorldStr2,'+',CorOrgStr{8}, '-(', gSym{2},'+',affineyStr, ') )' ];
  
  nLpStr1 = ['(( (',diffStr{1},'*',cosStr,'+',diffStr{2},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr1 = [nLpStr1 , ' + ' , ...
             [ '( (-',diffStr{1},'*',sinStr,'+',diffStr{2},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
  nLpStr2 = ['(( (',diffStr{3},'*',cosStr,'+',diffStr{4},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr2 = [nLpStr2 , ' + ' , ...
             [ '( (-',diffStr{3},'*',sinStr,'+',diffStr{4},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
           
  nLpStr3 = ['(( (',diffStr{5},'*',cosStr,'+',diffStr{6},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr3 = [nLpStr3 , ' + ' , ...
             [ '( (-',diffStr{5},'*',sinStr,'+',diffStr{6},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
  nLpStr4 = ['(( (',diffStr{7},'*',cosStr,'+',diffStr{8},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr4 = [nLpStr4 , ' + ' , ...
             [ '( (-',diffStr{7},'*',sinStr,'+',diffStr{8},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];

  %--Four robot corners and obstacle body constratint
  
  cosStr = ['cos(', num2str(obs(3)), ')'];
  sinStr = ['sin(', num2str(obs(3)), ')'];
  xradStr = ['(', num2str(oscalingV(1)), ')'];
  yradStr = ['(', num2str(oscalingV(2)), ')'];
  powStr = ['(', num2str(powerV(2)), ')'];
  pWorldStr1 = ['(', gSym{1},'+',affinexStr, ')'];
  pWorldStr2 = ['(', gSym{2},'+',affineyStr, ')'];
  
  CorOrgStr{1} =['(',CorStr{1},'*',cosStr,'-',CorStr{2},'*',sinStr,')'];
  CorOrgStr{2} =['(',CorStr{1},'*',sinStr,'+',CorStr{2},'*',cosStr,')'];
  CorOrgStr{3} =['(',CorStr{3},'*',cosStr,'-',CorStr{4},'*',sinStr,')'];
  CorOrgStr{4} =['(',CorStr{3},'*',sinStr,'+',CorStr{4},'*',cosStr,')'];
  CorOrgStr{5} =['(',CorStr{5},'*',cosStr,'-',CorStr{6},'*',sinStr,')'];
  CorOrgStr{6} =['(',CorStr{5},'*',sinStr,'+',CorStr{6},'*',cosStr,')'];
  CorOrgStr{7} =['(',CorStr{7},'*',cosStr,'-',CorStr{8},'*',sinStr,')'];
  CorOrgStr{8} =['(',CorStr{7},'*',sinStr,'+',CorStr{8},'*',cosStr,')'];
  
  diffStr{1} = [ '(',pWorldStr1,'+',CorOrgStr{1}, '-(', num2str(obs(1)), '))' ];
  diffStr{2} = [ '(',pWorldStr2,'+',CorOrgStr{2}, '-(', num2str(obs(2)), '))' ];
  diffStr{3} = [ '(',pWorldStr1,'+',CorOrgStr{3}, '-(', num2str(obs(1)), '))' ];
  diffStr{4} = [ '(',pWorldStr2,'+',CorOrgStr{4}, '-(', num2str(obs(2)), '))' ];
  diffStr{5} = [ '(',pWorldStr1,'+',CorOrgStr{5}, '-(', num2str(obs(1)), '))' ];
  diffStr{6} = [ '(',pWorldStr2,'+',CorOrgStr{6}, '-(', num2str(obs(2)), '))' ];
  diffStr{7} = [ '(',pWorldStr1,'+',CorOrgStr{7}, '-(', num2str(obs(1)), '))' ];
  diffStr{8} = [ '(',pWorldStr2,'+',CorOrgStr{8}, '-(', num2str(obs(2)), '))' ];
  
  nLpStr5 = ['(( (',diffStr{1},'*',cosStr,'+',diffStr{2},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr5 = [nLpStr5 , ' + ' , ...
             [ '( (-',diffStr{1},'*',sinStr,'+',diffStr{2},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
  nLpStr6 = ['(( (',diffStr{3},'*',cosStr,'+',diffStr{4},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr6 = [nLpStr6 , ' + ' , ...
             [ '( (-',diffStr{3},'*',sinStr,'+',diffStr{4},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
           
  nLpStr7 = ['(( (',diffStr{5},'*',cosStr,'+',diffStr{6},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr7 = [nLpStr7 , ' + ' , ...
             [ '( (-',diffStr{5},'*',sinStr,'+',diffStr{6},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
  nLpStr8 = ['(( (',diffStr{7},'*',cosStr,'+',diffStr{8},'*',sinStr,')', ...
               '/', xradStr, ' )^', powStr ];
  nLpStr8 = [nLpStr8 , ' + ' , ...
             [ '( (-',diffStr{7},'*',sinStr,'+',diffStr{8},'*',cosStr,')' ...
               '/',yradStr,' )^',powStr, ')^(1/',powStr,')' ] ];
%--Sum of 8 constraint
  nLpStr1
  nLpStr2
  nLpStr3
  nLpStr4
  nLpStr5
  nLpStr6
  nLpStr7
  nLpStr8
  bc = constraint(1, nLpStr1, Inf, 'trajectory',gSym)...        % Obs NE corner
         + constraint(1,nLpStr2,Inf, 'trajectory',gSym)...      % Obs SE corner
         + constraint(1,nLpStr3,Inf, 'trajectory',gSym)...      % Obs SW corner
         + constraint(1,nLpStr4,Inf, 'trajectory',gSym)...      % Obs NW corner
         + constraint(1,nLpStr5,Inf, 'trajectory',gSym)...       % Rob NE corner
         + constraint(1,nLpStr6,Inf, 'trajectory',gSym)...       % Rob SE corner
         + constraint(1,nLpStr7,Inf, 'trajectory',gSym)...       % Rob SW corner
         + constraint(1,nLpStr8,Inf, 'trajectory',gSym);         % Rob NW corner
  end
  
  


%========================== RecBodyAvoidRectLp ==========================
  %
  % @brief  Create a constraint for rectangular obstacle with a equal
  % distance constraint ||rv||=||v|| and minimize ||v|| in order to find
  % the possible contact location.
  %
  % @param[in]  gSym    Symbolic description of state vector.
  % @param[in]  obs Symbolic description of obstacle shape and orientation.
  % @param[in]  scale   Ratio between the edges of obstacle (scalar or 2-vector).
  % @param[in]  power   Order of Lp constraint approximating the sqyare.
  % @param[in]  rad    Radius of the robot.
  %
  function bc = RectBodyAvoidRectLp(gSym, rscale, obs, oscale, power)

      epsilonn=1e-13;
      epsilon_optimality = 1e-10;
  if isscalar(rscale) % Scaling for robot
      rscalingV= [rscale rscale];
  else
      rscalingV= [rscale(1) rscale(2)];        
  end
  if isscalar(oscale) % Scaling for obstacle
      oscalingV= [oscale pscale];
  else
      oscalingV= [oscale(1) oscale(2)];
          
  end
  if isscalar(power) % Order of Robot and Obstacle
      powerV= [power power];
  else
      powerV= [power(1) power(2)];
          
  end
  
  %--Four robot corners and obstacle body constratint
  
  cosStr = ['cos(', gSym{3}, '-(', num2str(obs(3)),'))'];
  sinStr = ['sin(', gSym{3}, '-(', num2str(obs(3)),'))'];
  ocosStr =['cos(', num2str(obs(3)),')'];
  osinStr =['sin(', num2str(obs(3)),')'];
  rcosStr =['cos(', gSym{3},')'];
  rsinStr =['sin(', gSym{3},')'];
  rxradStr = ['(', num2str(rscalingV(1)), ')'];
  ryradStr = ['(', num2str(rscalingV(2)), ')'];
  xradStr = ['(', num2str(oscalingV(1)), ')'];
  yradStr = ['(', num2str(oscalingV(2)), ')'];
  rpowStr = ['(', num2str(powerV(1)), ')'];
  powStr = ['(', num2str(powerV(2)), ')'];
  
  rvStr{1} = ['(',cosStr,'*', gSym{4},'-',sinStr,'*',gSym{5},'+', ocosStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', osinStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{2} = ['(',sinStr,'*', gSym{4},'+',cosStr,'*',gSym{5},'-', osinStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', ocosStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{3} = ['( ', ocosStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', osinStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{4} = ['(-', osinStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', ocosStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{5} = ['( ', rcosStr,'*((', num2str(obs(1)), ')-',gSym{1},')+', rsinStr,'*((', num2str(obs(2)), ')-',gSym{2},'))'];
  rvStr{6} = ['(-', rsinStr,'*((', num2str(obs(1)), ')-',gSym{1},')+', rcosStr,'*((', num2str(obs(2)), ')-',gSym{2},'))'];
  
  CorStr{1} = [ '((', gSym{5}, '/',ryradStr,')^(',rpowStr ,'  ))' ]; % (v2/alpha2)^p1
  CorStr{2} = [ '(-(', gSym{4}, '/',rxradStr,')^(',rpowStr ,'-1)*(',gSym{5},'/',rxradStr,'))' ]; % -(v1/alpha1)^{p1-1}v2/alpha1
  CorStr{3} = [ '(-(', gSym{5}, '/',ryradStr,')^(',rpowStr ,'-1)*(',gSym{4},'/',ryradStr,'))' ]; % -(v2/alpha2)^{p1-1}v1/alpha1
  CorStr{4} = [ '((', gSym{4}, '/',rxradStr,')^(',rpowStr ,'  ))' ]; % (v1/alpha1)^p1
  CorStr{5} = [ '((', rvStr{1}, '/',xradStr,')^(',powStr ,'-1)/',xradStr,')' ]; % (rv1/beta1)^{p2-1}/beta1
  CorStr{6} = [ '((', rvStr{2}, '/',yradStr,')^(',powStr ,'-1)/',yradStr,')' ]; % (rv2/beta1)^{p2-1}/beta2
  CorStr{7} = [ '(', rvStr{1}, '/',xradStr,')^(',powStr ,')' ]; % (rv1/beta1)^{p2}
  CorStr{8} = [ '(', rvStr{2}, '/',yradStr,')^(',powStr ,')' ]; % (rv2/beta2)^{p2}
  CorStr{9} = [ '(', rvStr{3}, '/',xradStr,')^(',powStr ,')' ]; % (rv1/beta1)^{p2}--> Center to Center
  CorStr{10} = [ '(', rvStr{4}, '/',yradStr,')^(',powStr ,')' ]; % (rv2/beta2)^{p2}
  CorStr{11} = [ '(', rvStr{5}, '/',rxradStr,')^(',rpowStr ,')' ]; % (v1/alpha1)^{p1} --> Center to Center
  CorStr{12} = [ '(', rvStr{6}, '/',ryradStr,')^(',rpowStr ,')' ]; % (v2/alpha2)^{p1}
  
  CorOrgStr{1} =['(',CorStr{1},'*',cosStr,'-',CorStr{2},'*',sinStr,')'];
  CorOrgStr{2} =['(',CorStr{1},'*',sinStr,'+',CorStr{2},'*',cosStr,')'];
  CorOrgStr{3} =['(',CorStr{3},'*',cosStr,'-',CorStr{4},'*',sinStr,')'];
  CorOrgStr{4} =['(',CorStr{3},'*',sinStr,'+',CorStr{4},'*',cosStr,')'];
  CorOrgStr{5} =['(',CorOrgStr{1},'*',CorStr{5},'+',CorOrgStr{2},'*',CorStr{6},')'];    %necessary condition
  CorOrgStr{6} =['(',CorOrgStr{3},'*',CorStr{5},'+',CorOrgStr{4},'*',CorStr{6},')'];
  CorOrgStr{7} =['(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')-(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')'];  % 1-dim Manifold (Hyper surface)
  CorOrgStr{8} =['(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')'];   % Min distance
  CorOrgStr{9} =['(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')'];   % Min distance
  CorOrgStr{10} =['(', CorStr{9},'+',  CorStr{10},')^(1/',powStr,')-(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')'];   % Comparison to center to center
  CorOrgStr{11} =['(', CorStr{11},'+',  CorStr{12},')^(1/',rpowStr,')-(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')']; %Comparison to center to center
  CorOrgStr{12} =['(', CorStr{9},'+',  CorStr{10},')^(1/',powStr,')'];   % center to center
  
  nLpStr1 = CorOrgStr{5};           % combine cells into one string
  nLpStr2 = CorOrgStr{6};
  nLpStr3 = CorOrgStr{7};           % ||v||=||rv||
  nLpStr4 = CorOrgStr{8};           % ||v||>1
  nLpStr5 = CorOrgStr{9};           % ||rv||>1
  nLpStr6 = CorOrgStr{10};           % ||v||<||vc|| Center distance
  nLpStr7 = CorOrgStr{11};           % ||rv||<||rvc|| Center distance
  nLpStr8 = CorOrgStr{12};           % 1<||rvc|| Center distance
%--Sum of 4 constraint
  nLpStr1
  nLpStr2
  nLpStr3
  nLpStr4
  nLpStr5
  nLpStr6
  nLpStr7
  nLpStr8
  
  bc = constraint(1+epsilonn,nLpStr4,Inf, 'trajectory',gSym)...         % Safety condition
         +  constraint(-epsilonn,nLpStr3,epsilonn, 'trajectory',gSym)...      % Hyper surface               % 
         + constraint(epsilonn,nLpStr6,Inf, 'trajectory',gSym)...
         + constraint(epsilonn,nLpStr7,Inf, 'trajectory',gSym) ...         % Safety condition
          + constraint(-epsilon_optimality, nLpStr1, epsilon_optimality, 'trajectory',gSym)...        % Necessary condition
          + constraint(-epsilon_optimality,nLpStr2,epsilon_optimality, 'trajectory',gSym);      % Necessary condition     

           
  end

  %========================== RecBodyAvoidRect ==========================
  %
  % @brief  Create a constraint for rectangular obstacle with a unit
  % distance constraint ||v||=1 and minimize ||rv|| in order to find
  % the exact closest point on the robot boundary
  %
  % @param[in]  gSym    Symbolic description of state vector.
  % @param[in]  obs Symbolic description of obstacle shape and orientation.
  % @param[in]  scale   Ratio between the edges of obstacle (scalar or 2-vector).
  % @param[in]  power   Order of Lp constraint approximating the sqyare.
  % @param[in]  rad    Radius of the robot.
  %
  function bc = RectBodyAvoidRectLp2(gSym, rscale, obs, oscale, power)

      epsilonn=1e-10;
      epsilon_optimality = 1e-10;
  if isscalar(rscale) % Scaling for robot
      rscalingV= [rscale rscale];
  else
      rscalingV= [rscale(1) rscale(2)];        
  end
  if isscalar(oscale) % Scaling for obstacle
      oscalingV= [oscale pscale];
  else
      oscalingV= [oscale(1) oscale(2)];
          
  end
  if isscalar(power) % Order of Robot and Obstacle
      powerV= [power power];
  else
      powerV= [power(1) power(2)];
          
  end
  
  %--Four robot corners and obstacle body constratint
  
  cosStr = ['cos(', gSym{3}, '-(', num2str(obs(3)),'))'];
  sinStr = ['sin(', gSym{3}, '-(', num2str(obs(3)),'))'];
  ocosStr =['cos(', num2str(obs(3)),')'];
  osinStr =['sin(', num2str(obs(3)),')'];
  rcosStr =['cos(', gSym{3},')'];
  rsinStr =['sin(', gSym{3},')'];
  rxradStr = ['(', num2str(rscalingV(1)), ')'];
  ryradStr = ['(', num2str(rscalingV(2)), ')'];
  xradStr = ['(', num2str(oscalingV(1)), ')'];
  yradStr = ['(', num2str(oscalingV(2)), ')'];
  rpowStr = ['(', num2str(powerV(1)), ')'];
  powStr = ['(', num2str(powerV(2)), ')'];
  
  rvStr{1} = ['(',cosStr,'*', gSym{4},'-',sinStr,'*',gSym{5},'+', ocosStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', osinStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{2} = ['(',sinStr,'*', gSym{4},'+',cosStr,'*',gSym{5},'-', osinStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', ocosStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{3} = ['( ', ocosStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', osinStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{4} = ['(-', osinStr,'*(',gSym{1},'-(', num2str(obs(1)), '))+', ocosStr,'*(',gSym{2},'-(', num2str(obs(2)), ')))'];
  rvStr{5} = ['( ', rcosStr,'*((', num2str(obs(1)), ')-',gSym{1},')+', rsinStr,'*((', num2str(obs(2)), ')-',gSym{2},'))'];
  rvStr{6} = ['(-', rsinStr,'*((', num2str(obs(1)), ')-',gSym{1},')+', rcosStr,'*((', num2str(obs(2)), ')-',gSym{2},'))'];
  
  CorStr{1} = [ '((', gSym{5}, '/',ryradStr,')^(',rpowStr ,'  ))' ]; % (v2/alpha2)^p1
  CorStr{2} = [ '(-(', gSym{4}, '/',rxradStr,')^(',rpowStr ,'-1)*(',gSym{5},'/',rxradStr,'))' ]; % -(v1/alpha1)^{p1-1}v2/alpha1
  CorStr{3} = [ '(-(', gSym{5}, '/',ryradStr,')^(',rpowStr ,'-1)*(',gSym{4},'/',ryradStr,'))' ]; % -(v2/alpha2)^{p1-1}v1/alpha1
  CorStr{4} = [ '((', gSym{4}, '/',rxradStr,')^(',rpowStr ,'  ))' ]; % (v1/alpha1)^p1
  CorStr{5} = [ '((', rvStr{1}, '/',xradStr,')^(',powStr ,'-1)/',xradStr,')' ]; % (rv1/beta1)^{p2-1}/beta1
  CorStr{6} = [ '((', rvStr{2}, '/',yradStr,')^(',powStr ,'-1)/',yradStr,')' ]; % (rv2/beta1)^{p2-1}/beta2
  CorStr{7} = [ '(', rvStr{1}, '/',xradStr,')^(',powStr ,')' ]; % (rv1/beta1)^{p2}
  CorStr{8} = [ '(', rvStr{2}, '/',yradStr,')^(',powStr ,')' ]; % (rv2/beta2)^{p2}
  CorStr{9} = [ '(', rvStr{3}, '/',xradStr,')^(',powStr ,')' ]; % (rv1/beta1)^{p2}--> Center to Center
  CorStr{10} = [ '(', rvStr{4}, '/',yradStr,')^(',powStr ,')' ]; % (rv2/beta2)^{p2}
  CorStr{11} = [ '(', rvStr{5}, '/',rxradStr,')^(',rpowStr ,')' ]; % (v1/alpha1)^{p1} --> Center to Center
  CorStr{12} = [ '(', rvStr{6}, '/',ryradStr,')^(',rpowStr ,')' ]; % (v2/alpha2)^{p1}
  
  CorOrgStr{1} =['(',CorStr{1},'*',cosStr,'-',CorStr{2},'*',sinStr,')'];
  CorOrgStr{2} =['(',CorStr{1},'*',sinStr,'+',CorStr{2},'*',cosStr,')'];
  CorOrgStr{3} =['(',CorStr{3},'*',cosStr,'-',CorStr{4},'*',sinStr,')'];
  CorOrgStr{4} =['(',CorStr{3},'*',sinStr,'+',CorStr{4},'*',cosStr,')'];
  CorOrgStr{5} =['(',CorOrgStr{1},'*',CorStr{5},'+',CorOrgStr{2},'*',CorStr{6},')'];    %necessary condition
  CorOrgStr{6} =['(',CorOrgStr{3},'*',CorStr{5},'+',CorOrgStr{4},'*',CorStr{6},')'];
  CorOrgStr{7} =['(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')-(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')'];  % 1-dim Manifold (Hyper surface)
  CorOrgStr{8} =['(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')'];   % Min distance
  CorOrgStr{9} =['(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')'];   % Min distance
  CorOrgStr{10} =['(', CorStr{9},'+',  CorStr{10},')^(1/',powStr,')-(', CorStr{7},'+',  CorStr{8},')^(1/',powStr,')'];   % Comparison to center to center
  CorOrgStr{11} =['(', CorStr{11},'+',  CorStr{12},')^(1/',rpowStr,')-(', CorStr{1},'+',  CorStr{4},')^(1/',rpowStr,')']; %Comparison to center to center
  CorOrgStr{12} =['(', CorStr{9},'+',  CorStr{10},')^(1/',powStr,')'];   % center to center
  
  nLpStr1 = CorOrgStr{5};           % combine cells into one string
  nLpStr2 = CorOrgStr{6};
  nLpStr3 = CorOrgStr{7};           % ||v||=||rv||
  nLpStr4 = CorOrgStr{8};           % ||v||>1
  nLpStr5 = CorOrgStr{9};           % ||rv||>1
  nLpStr6 = CorOrgStr{10};           % ||v||<||vc|| Center distance
  nLpStr7 = CorOrgStr{11};           % ||rv||<||rvc|| Center distance
%--Sum of 4 constraint
  nLpStr1
  nLpStr2
  nLpStr3
  nLpStr4
  nLpStr5
  nLpStr6
  nLpStr7

  
  bc = constraint(1+epsilonn,nLpStr5,Inf, 'trajectory',gSym)...         % Safety condition
         +  constraint(1,nLpStr4,1+epsilonn, 'trajectory',gSym)...      % Hyper surface               % 
         + constraint(epsilonn,nLpStr6,Inf, 'trajectory',gSym)...
         + constraint(-epsilon_optimality, nLpStr1, epsilon_optimality, 'trajectory',gSym)...        % Necessary condition
          + constraint(-epsilon_optimality,nLpStr2,epsilon_optimality, 'trajectory',gSym);      % Necessary condition     

           
  end
  
  
end  
%)
%

end


%
%================================ planarSE ===============================
