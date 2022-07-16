function [dx] = dynamic(t,x,Model_data)

b=Model_data.b;
u=u_star_func(t,Model_data);

dx=x;
dx(1)=b(t)*u;
end

