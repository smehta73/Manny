%============================== OPTRAGEN_IPOPT =============================
%
% @brief    Interface between OPTRAGEN and IPOPT. 
%
% @argin    ghSnopt     : Function handle generated from
%                         snoptFunction_IPOPT.m
%           nobj        : Number of cost function (Usually only one cost
%                         function, nobj=1)
%           nFreeVar    : Number of free variables
%           nConstraint : Number of constraints for each free varaible
%           init        : Initial Guess
%           xlow        : Lower limit of the free variables
%           xupp        : Upper limit of the free variables
%           cl          : Lower limit of the constraints
%           cu          : Upper limit of the constraints 
% 
%
% REMARK: 
%  1) The sparse structure is created based on the Jacobian matrix of the
%  initial condition.
%  2) The structure (ordering) of the constraint follows the original
%  arguments using SNOPT. (e.g., cl=[nlp.LinCon.lb;nlp.nlb], and cu=[nlp.LinCon.ub;nlp.nub]
%
%=============================== OPTRAGEN_IPOPT ==============================

%
% @file     OPTRAGEN_IPOPT.m
%
% @author   Nak-seung Patrick Hyun,     nhyun3@gatech.edu
%           
% @date     2016/11/28 [created]
%
%=============================== OPTRAGEN_IPOPT ==============================

function [x, info] = OPTRAGEN_IPOPT(ghSnopt,nobj,nFreeVar,nConstraint,init,xlow,xupp,cl,cu)

x0 = init';
options.lb = xlow;
options.ub = xupp;
options.cl = cl;
options.cu = cu;

options.ipopt.mu_strategy = 'adaptive';

funcs.objective         = @eval_f;
funcs.constraints       = @eval_g;
funcs.gradient          = @eval_grad_f;
funcs.jacobian          = @eval_jac_g;
funcs.jacobianstructure = @eval_jac_g_struct;

options.ipopt.hessian_approximation = 'limited-memory';

[x, info] = ipopt(x0, funcs, options);


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Evaluate value of objective function
  function f = eval_f(x)

    [F,~,~,~,~]=ghSnopt(x);
    f=F(1:nobj);

  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Evaluate gradient of objective function
  function df = eval_grad_f(x)

    [~,G,~,~,~]=ghSnopt(x);
    df=G(1:nobj,:);

  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Evaluate value of constraint bodies
  function g = eval_g(x)

    [F,~,~,~,~]=ghSnopt(x);
    g=F(nobj+1:end);

  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Return constraint Jacobian strcture
  function A = eval_jac_g_struct

   
    
%      [~,G,~,~,~]=ghSnopt(init');
%      A=sparse(G(nobj+1:end,:));
         B=ones(nConstraint,nFreeVar);
         A=sparse(B);
      
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Evaluate constraint Jacobian
  function A = eval_jac_g(x)
    
      [~,G,~,~,~]=ghSnopt(x);
      A=sparse(G(nobj+1:end,:));
      
      
  end

end