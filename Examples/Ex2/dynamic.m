function [dx] = dynamic(t,x,params)

u_star_func=params.u_star_func;
dx=x;
dx(1)=x(2);
dx(2)=-x(1)+u_star_func(t);

end

