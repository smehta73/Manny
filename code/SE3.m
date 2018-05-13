%================================== SE3 ==================================
%
%  function g = SE3(d, R)
%
%
%  Generates an instance of the class object SE3.  As a class, it acts as
%  though it were a Matlab variable.  Different operations and actions
%  can be applied to it.
%
%
%================================== SE3 ==================================
classdef SE3 < handle
    

%============================ Member Variables ===========================
%

properties (Access = protected)
  M
end

%
%============================= Public Methods ============================
%
%(

methods 
  function g = SE3(d, R)
    if (nargin == 0)
      d = zeros([3 1]);
      R = eye(3);
    elseif ( (size(d,1) == 1) && (size(d,2) == 3) )		% if row vector.
      d = d';						%   make column.
    elseif ( (size(d,1) ~= 3) || (size(d,2) ~= 1) )
      error('The homogeneous group element incorrect dimensions');
    end
    g.M = [R, d;0 0 0 1];
  % obj = class(g, 'SE3');
  end
  
      
% function plotcam(g, label, linecolor, sc)
%
%  Plots the coordinate frame associated to g.  The figure is cleared, 
%  so this will clear any existing graphic in the figure.  To plot on
%  top of an existing figure, set hold to on.  The label is the name
%  of label given to the frame (if given is it writen out).  The 
%  linecolor is a valid plot linespec character.  Finally sc is the
%  specification of the scale for plotting.  It will rescale the
%  line segments associated with the frame axes and also with the location
%  of the label, if there is a label.
%
%  Inputs:
%    g		- The SE3 coordinate frame to plot.
%    label	- The label to assign the frame.
%    linecolor  - The line color to use for plotting.  (See `help plot`) 
%    sc		- scale to plot things at.
%		  a 2x1 vector, first element is length of axes.
%		    second element is a scalar indicating roughly how far
%		    from the origin the label should be placed.
%    Psi	- intrinsic camera matrix for plotting virtual image plane.
%    imdims	- image dimensions for plotting virtual image plane.
%
%  Output:
%    The coordinate frame, and possibly a label, is plotted.
%
%================================== plot =================================
  function coord = plotCam(g, flabel, lcol, sc, Psi, imdims)  

    if ( (nargin < 2) )
      flabel = '';
    end
    
    if ( (nargin < 3) || isempty(lcol) )
      lcol = 'b';
    end

    if ( (nargin < 4) || isempty(sc) )
      sc = [1.0 0.5];
    elseif (size(sc,2) == 1)
      sc = [sc 2];
    end
    
    d = g.M(1:3,4);
    R = g.M(1:3,1:3);
    ex = R*[sc(1);0;0];				% get rotated x-axis.
    ey = R*[0;0.75*sc(1);0];		% get rotated y-axis.
    ez = R*[0;0;2*sc(1)];           % get rotated z-axis.
    
    isheld = ishold;
    pts = [d-ex , d+ex];
    
    plot3(pts(1,:), pts(2,:), pts(3,:),lcol);		% x-axis
    hold on;
    pts = [d-ey , d+ey];
    plot3(pts(1,:), pts(2,:), pts(3,:),lcol);		% y-axis
    pts = [d , d+ez];
    plot3(pts(1,:), pts(2,:), pts(3,:),lcol);		% z-axis

    plot3(d(1), d(2), d(3), [lcol 'o'],'MarkerSize',7);		% origin
    coord(1)=d(1);
    coord(2)=d(2);
    coord(3)=d(3);
    if (~isempty(flabel))
     pts = d - (sc(2)/sc(1))*(ex+ey+ez);
     text(pts(1), pts(2), pts(3),flabel);
    end

    if (nargin == 6)
    %-- Build fake image plane and truncated imaging cone (a square pyramid).
      imCorners = [1, 1, imdims(1), imdims(1), 1; 1, imdims(2), imdims(2), 1, 1;
               1, 1, 1        ,   1      , 1];
      rayCorners = sc(1)*inv(Psi)*imCorners;
      facesC = reshape([rayCorners; zeros(3,5); rayCorners], 3, 15);
      facesO = R * facesC + repmat(d, [1, size(facesC,2)]);
                                      % Gives faces of imaging cone.
  
      p1 = struct('vertices',facesO','faces',[1 4 2;2 4 7;2 7 10;2 10 1]);
      h1 = patch(p1);
      set(h1,'FaceColor',[52 217 160]/255, 'EdgeColor', 'r', 'FaceAlpha', [0.5]);
                                    % Display imaging code as patches.
  
      p2 = struct('vertices',facesO','faces',[1 10 7;7 4 1]);
      h2 = patch(p2);
      set(h2,'FaceColor',[247 239 7]/255,'EdgeColor', 'none', 'FaceAlpha', 0.25);
    end


    if (~isheld)
      hold off;
    end
  end
  
  
  %================================== plotObs =================================
%
%  function plotObs(g, label, linecolor, sc)
%
%  Plots the coordinate frame associated to g as though it were the central
%  observer from which all other configurations should be with relation to.  
%  The figure is overwritten, so this will clear any existing graphic in the 
%  figure.  To plot on top of an existing figure, set hold to on.  The label 
%  is the name of label given to the frame (if given is it writen out).  The 
%  linecolor is a valid plot linespec character.  Finally sc is the
%
%  Inputs:
%    g		- The SE3 coordinate frame to plot.
%    label	- The label to assign the frame.
%    linecolor  - The line color to use for plotting.  (See `help plot`) 
%    sc		- scale to plot things at.
%		  a 2x1 vector, first element is length of axes.
%		    second element is a scalar indicating roughly how far
%		    from the origin the label should be placed.
%
%  Output:
%    The coordinate frame, and possibly a label, is plotted.
%
%================================== plotObs =================================

  function plot(g, flabel, lcol, sc)   

    if ( (nargin < 2) )
      flabel = '';
    end

    if ( (nargin < 3) || isempty(lcol) )
      lcol = 'b';
    end

    if ( (nargin < 4) || isempty(sc) )
      sc = [1.0 0.5];
    elseif (size(sc,2) == 1)
      sc = [sc 2];
    end

    d = g.M(1:3,4);
    R = g.M(1:3,1:3);

    ex = R*[sc(1);0;0];		% get rotated x-axis.
    ey = R*[0;sc(1);0];		% get rotated y-axis.
    ez = R*[0;0;sc(1)];		% get rotated z-axis.

    isheld = ishold;

    lspec = [lcol '-.'];

    pts = [d-ex , d+ex];
    plot3(pts(1,:), pts(2,:), pts(3,:), lspec);		% x-axis
    hold on;
      pts = [d-ey , d+ey];
      plot3(pts(1,:), pts(2,:), pts(3,:), lspec);		% y-axis
      pts = [d-ez , d+ez];
      plot3(pts(1,:), pts(2,:), pts(3,:), lspec);		% z-axis

      plot3(d(1), d(2), d(3), [lcol 'o'],'MarkerSize',7);		% origin

    if (~isempty(flabel))
      pts = d - (sc(2)/sc(1))*(ex+ey+ez);
      text(pts(1), pts(2), pts(3),flabel);
    end

    if (~isheld)
      hold off;
    end

   axis equal;
  end

  %============================== leftact ==============================
  %
  %  p2 = leftact(g, p)
  %		with p a 3x1 specifying point coordinates.
  %            or a 4x1 homogeneous point coordinate.
  %
  %  p2 = leftact(g, v)
  %		with v a 4x1 specifying a homogeneous point velocity.
  %
  %  This function takes a change of coordinates and a point/velocity,
  %  and returns the transformation of that point/velocity under the change
  %  of coordinates.  
  %  
  %  Alternatively, one can think of the change of coordinates as a 
  %  transformation of the point to somewhere else, e.g., a displacement
  %  of the point.  It all depends on one's perspective of the
  %  operation/situation.
  %
  %============================== leftact ==============================
  function x2 = leftact(g, x, type)      

  if ( (size(x,1) == 3) )					% If three vector.
    x2 = g.M(1:3,:)*[x;ones(1,size(x,2))];	%  treat like a point.
  elseif ( (size(x,1) == 4) )				% else it is homogeneous.
    x2 = g.M*x;								%  do the right thing.
  end

  end
  
  %=============================== times ===============================
  %
  %  function p2 = times(g, p)
  %
  %
  %  This function is the operator overload that implements the left action
  %  of g on the point p, the homogeneous point p, or on the homogeneous 
  %  vector v.
  %
  %  Can also be typed as:  
  %    >> p2 = g.*p
  %    >> v2 = g.*v
  %
  %=============================== times ===============================
  function p2 = times(g, p)
    p2 = g.leftact(p);
  end
  
%================================== log ==================================
%
%  function xi = log(g, tau)
%
%  Take the logarithm of the group element g.  If the time period of
%  the action is not given, it is assumed to be unity.
%
%================================== log ==================================
  function xi = log(g, tau)

    if ( (nargin < 2) || isempty(tau) )
      tau = 1;
    end

    normw = acos( (trace(g.M(1:3,1:3))-1)/2 )/tau;
    if (normw == 0)				% If rotation, pure translation.

      w = zeros([3 1]);
      v = g.M(1:3,4)/tau;

    else					% else, use logarithm equation.

      %--(1) First do the log of the rotation matrix.
      hatw = (normw/(2*sin(normw*tau)))*(g.M(1:3,1:3) - (g.M(1:3,1:3))');
      w = [hatw(3,2) ; hatw(1,3) ; hatw(2,1)];

      %--(2) Second, use both hat omega and omega to get the velocity.
      v = (normw^2)*inv((eye(3) - g.M(1:3,1:3))*hatw + tau*w*w')*g.M(1:3,4);

    end


    %--(3) Concatenate the two vector elements to make one se2 Lie algebra element.
    xi = [v;w];

  end

  %================================ inv ================================
  %
  %  invg = inv(g)
  %  
  %  Computes and returns the inverse to g.
  %
  %================================ inv ================================
  function invg = inv(g)  

  invM = inv(g.M);
  invg = SE3(invM(1:3,end), invM(1:3,1:3));

  end
  
  function M = homog(g)

  M = g.M;
    
  end
  
  %============================= getTranslation ============================
  function d = getTranslation(g)
    d = g.M(1:3,4);
  end

  %============================== getRotation ==============================
  function R = getRotation(g)

    R = g.M(1:3,1:3);

  end
  
  %================================ display ================================
%
%  function display(g)
%
%
%  This is the default display function for the SE2 class.  It simply
%  displays the position followed by the rotation.
%
%================================ display ================================
  function display(g)

    if isequal(get(0,'FormatSpacing'),'compact')
      disp([inputname(1) ' =']);
      disp(g.M);
    else
      disp(' ');
      disp([inputname(1) ' =']);
      disp(' ');
      disp(g.M);
    end
  end
  
  %================================ Adjoint ================================
%
%  function g = adjoint(g1, g2)
%
%
%  Computes and returns the adjoint of g.  The adjoint is defined to
%  operate as:
%
%    Ad_g1 (g2) = g1 * g2 * inverse g1
%
%================================ Adjoint ================================
  function z = adjoint(g, x)

    if (nargin == 1)
      R = g.M(1:3,1:3);
      d = g.M(1:3,4);
      z = [R , SE3.hatso3(d)*R ; zeros(3) , R];
    elseif (isa(x,'SE3'))
      z = g.M*x*inv(g.M);
    elseif ( (size(x,1) == 6) && (size(x,2) == 1) )
      R = g.M(1:3,1:3);
      d = g.M(1:3,4);
      z = [R , SE3.hatso3(d)*R ; zeros(3) , R]*x;
    elseif ( (size(x,1) == 3) && (size(x,2) == 3) )
      z = g.M*x*inv(g.M);
    end

  end
  
%================================= mtimes ================================
%
%  function g = mtimes(g1, g2)
%
%
%  Computes and returns the product of g1 with g2.
%
%  Can also be typed as:  >> g3 = g1*g2
%
%================================= mtimes ================================
  function g = mtimes(g1, g2)

    g = SE3();
    g.M = g1.M*g2.M;

  end
  
  function texout(g, ftsr)

    if (nargin < 2)
      fstr = '%5.3f';
    end

    fprintf('\\SEhomogM{');
    for ii = 1:(size(g.M,1)-2)			% Print rotation matrix.
      for jj = 1:(size(g.M,2)-2)
        fprintf([ fstr ' & '], g.M(ii,jj));
      end
      fprintf([ fstr ' \\\\ \n'], g.M(ii,end-1));
    end
    for jj = 1:(size(g.M,2)-2)
      fprintf([ fstr ' & '], g.M(end-1,jj));
    end
    fprintf([ fstr ' } \n'], g.M(end-1,end-1));	
    fprintf('{'); 						% Print vector
    for ii = 1:(size(g.M,1)-2)
      fprintf([ fstr ' \\\\ '], g.M(ii,end));
    end
    fprintf([ fstr ' } \n'], g.M(end-1,end));	
    
  end




  
end


%
%======================== Hidden Member Functions ======================
%

methods (Hidden = true)
    
  %=============================== homogM2tex ==============================
%
%  function homogM2tex(M, fstr)
%
%  Output a Matlab matrix with homogeneous structure using latex format.  
%  Uses the \SEhomogM command in the gmech library.
%
%  INPUTS:
%    M      - The matlab matrix to output.
%    fstr   - Optional number to string conversion (default: '%5.3f')
%
%=============================== homogM2tex ==============================
  function homogM2tex(g, fstr)

    if (nargin < 2)
      fstr = '%5.3f';
    end

    fprintf('\\SEhomogM{');
    for ii = 1:(size(M,1)-2)			% Print rotation matrix.
      for jj = 1:(size(M,2)-2)
        fprintf([ fstr ' & '], M(ii,jj));
      end
      fprintf([ fstr ' \\\\ \n'], M(ii,end-1));
    end
    for jj = 1:(size(M,2)-2)
      fprintf([ fstr ' & '], M(end-1,jj));
    end
    fprintf([ fstr ' } \n'], M(end-1,end-1));	
    fprintf('{'); 						% Print vector
    for ii = 1:(size(M,1)-2)
      fprintf([ fstr ' \\\\ '], M(ii,end));
    end
    fprintf([ fstr ' } \n'], M(end-1,end));	
    
  end
end

%)
%
%============================= Static Methods ============================
%
%(

methods(Static, Hidden = true)

  function omegahat = hatso3(omega)

    omegahat = [0 , -omega(3) , omega(2); ...
            omega(3), 0, -omega(1); ...
	    -omega(2), omega(1), 0];

  end
 
end

methods(Static)

   %================================ hat ===============================
   %
   %  Takes a vector form of se(2) and hats it to get the homogeneous
   %  matrix form.
   %
   function xiHat = hat(xiVec)

   omegaHat = [0, -xiVec(6), xiVec(5); xiVec(6), 0, -xiVec(4); ...
               -xiVec(5), xiVec(4), 0];
   xiHat = [ omegaHat, xiVec(1:3); 0 0 0 0];

   end

   %=============================== unhat ==============================
   %
   %  Takes a vector form of se(2) and hats it to get the homogeneous
   %  matrix form.
   %
   function xiVec = unhat(xiHat)

   xiVec = [xiHat(1:3,3) ; xiHat(3,2); xiHat(1,3); xiHat(2,1)]; 

   end

   %================================ exp ===============================
   %
   %  Computes the exponential of a twist in se(3).
   %
   function gexp = exp(xi, tau)

   if (size(xi,2) == 1)
     xi = SE3.hat(xi);
   end

   if (nargin < 2)
     tau = 1;
   end

   expMat = expm(xi*tau);
   gexp = SE3(expMat(1:3,4), expMat(1:3,1:3));

   end


end
%)  [Static Methods]
  
end


%
%================================== SE3 ==================================
