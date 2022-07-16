 function [Dq_with_objective] = Dynamic_Real(t,q_scaled_with_obj,U_scaled,Auxdata)
q_scaled=q_scaled_with_obj.';
U1_scaled=U_scaled(1);
%% Scaled data
A_scaled=Auxdata.Model_data.A_scaled;
B_scaled=Auxdata.Model_data.B_scaled;
C_scaled=Auxdata.Model_data.C_scaled;
D_scaled=Auxdata.Model_data.D_scaled;

% q: unsclaed states
q=q_scaled;
for i=1:size(q_scaled,2)
    q(:,i)=A_scaled(i)+B_scaled(i,i)*q_scaled(:,i);
end

%unscaled controls
U1=C_scaled(1)+D_scaled(1,1)*U1_scaled;

%%
dq=zeros(numel(t),Auxdata.intermediateData.number_of_states);

dq(:,1)=q(:,2);
dq(:,2)=U1; %[W/W]
%dq(:,3)=1/2*(U1.^2+10000*(t>=0.98)*((q(end,2)+1).^2+1000*q(end,1).^2));
dq(:,3)=1/2*(U1.^2);

Dq=dq;
Dq_scaled(:,1)=B_scaled(1,1)^-1*Dq(:,1);
Dq_scaled(:,2)=B_scaled(2,2)^-1*Dq(:,2);
Dq_scaled(:,3)=B_scaled(3,3)^-1*Dq(:,3);

Dq_with_objective=Dq_scaled.';
end
