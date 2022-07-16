function [path_vecot_constraint] = Path_constraint(t,q_scaled,auxdata)
path_vecot_constraint=q_scaled(2:end,1)-1;
path_vecot_constraint=[];
end

