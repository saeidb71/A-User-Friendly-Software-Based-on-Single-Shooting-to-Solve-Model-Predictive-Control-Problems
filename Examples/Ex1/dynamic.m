function [dx] = dynamic(t,x,params)

l=params.l;

dx=x;
dx(1)=x(2);
dx(2)=u_star_func(t,l);

end

