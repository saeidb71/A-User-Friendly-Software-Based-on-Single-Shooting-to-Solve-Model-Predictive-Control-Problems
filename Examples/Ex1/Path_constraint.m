function [path_vecot_constraint] = Path_constraint(t,q_scaled,auxdata)


A_scaled=auxdata.Model_data.A_scaled;
B_scaled=auxdata.Model_data.B_scaled;
C_scaled=auxdata.Model_data.C_scaled;
D_scaled=auxdata.Model_data.D_scaled;

% q: unsclaed states
q=q_scaled;
for i=1:size(q_scaled,2)
    q(:,i)=A_scaled(i)+B_scaled(i,i)*q_scaled(:,i);
end


if t(end)==1
    path_vecot_constraint=[q(end,1) -q(end,1) q(end,2)+1 -q(end,2)-1].';
else
    path_vecot_constraint=[];
end
end

