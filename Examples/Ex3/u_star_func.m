function [u_start_exact] = u_star_func(t,Model_data)

a=Model_data.a;
b=Model_data.b;
x_tf_star=Model_data.x_tf_star;

u=a.^2*b(t)*x_tf_star;

u_start_exact=-min(1,max(-1,u));
end

