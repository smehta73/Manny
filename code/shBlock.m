%-------------------------------- shBlock --------------------------------
%
%  function blockh = shBlock(g, blens, colors, transp, drawLines)
%
%
%
%  Inputs:
%    g			- SE(3) description of block location.
%    blens		- Lengths of the block sides (along body x, y, and z axes)
%    colors		- Vector describing the color of the faces [1x3 vector]
%					If desired, the edge color can be specified.  In 
%					this case, then input is a [2x3] vector.
%					Default is same as face color.
%    transp		- The transparency level of the block.
%    drawLines	- Draw the edge lines?  [default is yes]
%					This overrides the colors specification.
%
%-------------------------------- shBlock --------------------------------

%
%  Name:		shBlock.m
%
%  Author:		Patricio A. Vela, 				pvela@gatech.edu
%
%  Created:		2011/05/11
%  Modified:	2011/05/11
%
%-------------------------------- shBlock --------------------------------
function blockh = block(g, blens, color, transp, drawLines)

if (size(color,1) == 1)
  color = repmat(color,[2,1]);
end

if (nargin <  5)
  drawLines = true;
end

if (nargin < 4)
  transp = [1 1];
elseif (isscalar(transp))
  transp = [transp transp];
end

facepts = blens([1, 1, 1, 1; 2, 2, 2, 2; 3, 3, 3, 3])/2;

f{1} = facepts .* [ -1, -1,  1,  1 ; -1, -1, -1, -1 ;  1, -1, -1,  1 ];
f{2} = facepts .* [  1,  1,  1,  1 ; -1, -1,  1,  1 ; -1,  1,  1, -1 ];
f{3} = facepts .* [  1,  1, -1, -1 ;  1,  1,  1,  1 ;  1, -1, -1,  1 ];
f{4} = facepts .* [ -1, -1, -1, -1 ;  1,  1, -1, -1 ;  1, -1, -1,  1 ];
f{5} = facepts .* [ -1, -1,  1,  1 ;  1, -1, -1,  1 ;  1,  1,  1,  1 ];
f{6} = facepts .* [ -1, -1,  1,  1 ;  1, -1, -1,  1 ; -1, -1, -1, -1 ];

hold on;
for ii=1:6
  f{ii} = g .* [f{ii}; ones(1, 4)];
  f{ii} = transpose(f{ii}(1:3,:));
  fh(ii) = fill3(f{ii}(:,1), f{ii}(:,2), f{ii}(:,3), color(1,:));

  if (drawLines)
    set(fh(ii),'EdgeColor',color(2,:));
  else
    set(fh(ii),'LineStyle','none');
  end
  set(fh(ii),'EdgeAlpha', transp(1), 'FaceAlpha', transp(2));
end
hold off;

if (nargout)
  blockh = fh;
end

end

%
%-------------------------------- shBlock --------------------------------
