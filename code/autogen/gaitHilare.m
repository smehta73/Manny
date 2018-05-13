function och = gaitHilare()

och.icf = @gaitHilare_icf; 
och.tcf = @gaitHilare_tcf; 
och.fcf = @gaitHilare_fcf; 

och.nlicf = @gaitHilare_nlicf; 
och.nltcf = @gaitHilare_nltcf; 
och.nlfcf = @gaitHilare_nlfcf; 
och.nlgcf = @gaitHilare_nlgcf; 



function [f,df] = gaitHilare_icf(x,xd,y,yd,th,thd,v)
 


end


function [f,df] = gaitHilare_tcf(x,xd,y,yd,th,thd,v)
 
	 f(1,:) = xd.^2+yd.^2;

	 df(1,:)= zeros(size(x)); 
	 df(2,:)= 2.*xd; 
	 df(3,:)= zeros(size(y)); 
	 df(4,:)= 2.*yd; 
	 df(5,:)= zeros(size(th)); 
	 df(6,:)= zeros(size(thd)); 
	 df(7,:)= zeros(size(v)); 

end


function [f,df] = gaitHilare_fcf(x,xd,y,yd,th,thd,v)
 


end


function [f,df] = gaitHilare_nlicf(x,xd,y,yd,th,thd,v)
 


end


function [f,df] = gaitHilare_nltcf(x,xd,y,yd,th,thd,v)
 
	 f(1,:) = ( (x- (-1) )./(0.65) ).^8 + ( (y- (0) )./(0.65) ).^8;
	 f(2,:) = xd-0.079577.*v.*cos(th);
	 f(3,:) = yd-0.079577.*v.*sin(th);

	 df(1,1,:) = 12.307692307692307692307692307692.*(1.5384615384615384615384615384615.*x + 1.5384615384615384615384615384615).^7; 
	 df(2,1,:) = zeros(size(xd)); 
	 df(3,1,:) = 251.06324271928456645682711758504.*y.^7; 
	 df(4,1,:) = zeros(size(yd)); 
	 df(5,1,:) = zeros(size(th)); 
	 df(6,1,:) = zeros(size(thd)); 
	 df(7,1,:) = zeros(size(v)); 
	 df(1,2,:) = zeros(size(x)); 
	 df(2,2,:) = 1.0.*ones(size(xd)); 
	 df(3,2,:) = zeros(size(y)); 
	 df(4,2,:) = zeros(size(yd)); 
	 df(5,2,:) = 0.079577.*v.*sin(th); 
	 df(6,2,:) = zeros(size(thd)); 
	 df(7,2,:) = -0.079577.*cos(th); 
	 df(1,3,:) = zeros(size(x)); 
	 df(2,3,:) = zeros(size(xd)); 
	 df(3,3,:) = zeros(size(y)); 
	 df(4,3,:) = 1.0.*ones(size(yd)); 
	 df(5,3,:) = -0.079577.*v.*cos(th); 
	 df(6,3,:) = zeros(size(thd)); 
	 df(7,3,:) = -0.079577.*sin(th); 

end


function [f,df] = gaitHilare_nlfcf(x,xd,y,yd,th,thd,v)
 


end


function [f,df] = gaitHilare_nlgcf(x,xd,y,yd,th,thd,v)
 


end


end