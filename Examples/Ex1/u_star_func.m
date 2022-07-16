function [u_start_exact] = u_star_func(t,l)
if t<3*l
    u_start_exact=-2/3/l*(1-t/3/l);
elseif t<1-3*l
    u_start_exact=0;
else
    u_start_exact=-2/3/l*(1-(1-t)/3/l);
end
end

