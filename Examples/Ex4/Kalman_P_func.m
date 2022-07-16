 function [dx] = Kalman_P_func(t,x,Auxdata)

kt=Auxdata.Model_data.kt; % N/m
mu=Auxdata.Model_data.mu; % kg
ms=Auxdata.Model_data.ms;
Q_Kalman=Auxdata.Kalman.Q_Kalman;

b1=Auxdata.Model_data.b1;
k1=Auxdata.Model_data.k1;

 P=reshape(x,[numel(Auxdata.Model_data.A_scaled)-1,numel(Auxdata.Model_data.A_scaled)-1]);
 F=[  0        1          0          0
     -kt/mu   -b1/mu     k1/mu      b1/mu
      0         -1         0         1
      0         b1/ms     -k1/ms    -b1/ms];
 
 dx=F*P+P*F.'+Q_Kalman;
 dx=dx(:);
end
