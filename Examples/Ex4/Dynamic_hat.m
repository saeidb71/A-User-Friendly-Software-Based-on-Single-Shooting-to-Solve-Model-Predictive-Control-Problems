 function [Dq_with_objective] = Dynamic_hat(t,q_scaled_with_obj,U_scaled,Auxdata)
q_scaled=q_scaled_with_obj.';
U1_scaled=U_scaled(1);
%% data

b1=Auxdata.Model_data.b1;
k1=Auxdata.Model_data.k1;

z0=Auxdata.Model_data.z0;
z0dot=Auxdata.Model_data.z0dot;

bt=Auxdata.Model_data.bt; % N/(m/s)
kt=Auxdata.Model_data.kt; % N/m
mu=Auxdata.Model_data.mu; % kg
ms=Auxdata.Model_data.ms; % kg

% objective function weights
w1=Auxdata.Model_data.w1;
w2=Auxdata.Model_data.w2;
w3=Auxdata.Model_data.w3;

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
Dq_scaled=dq;

dq(:,1)=q(:,2)-z0dot(t);
dq(:,2)=-kt/mu.*q(:,1)-b1/mu.*q(:,2)+k1/mu.*q(:,3)+b1/mu.*q(:,4)-1/mu.*U1;
dq(:,3)=-q(:,2)+q(:,4);
dq(:,4)=b1/ms.*q(:,2)-k1/ms.*q(:,3)-b1/ms.*q(:,4)+1/ms.*U1;  
dq(:,5)=w1*q(:,1).^2+w2*dq(:,4).^2+w3*U1.^2;

Dq=dq;
Dq_scaled(:,1)=B_scaled(1,1)^-1*Dq(:,1);
Dq_scaled(:,2)=B_scaled(2,2)^-1*Dq(:,2);
Dq_scaled(:,3)=B_scaled(3,3)^-1*Dq(:,3);
Dq_scaled(:,4)=B_scaled(4,4)^-1*Dq(:,4);
Dq_scaled(:,5)=B_scaled(5,5)^-1*Dq(:,5);

Dq_with_objective=Dq_scaled.';
end
