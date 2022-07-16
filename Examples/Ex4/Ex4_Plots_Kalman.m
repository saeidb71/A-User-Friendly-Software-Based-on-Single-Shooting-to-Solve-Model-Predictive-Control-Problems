clc;
clear;
close all;
%% 
load('Kalman_Mpc  , m: 50  , p: 50  , Ts: 0.01  , h: 0.01  , h_const: 0.01  , solver: 2  .mat');

output_number=1;
y_real=Kalman.H_Kalman*Unscaled_States_Out_Real(:,1:numel(Model_data.A_scaled)-1).';
y_hat=Kalman.H_Kalman*Unscaled_States_Out_hat(:,1:numel(Model_data.A_scaled)-1).';

figure()
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
hold on
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_hat(:,1))*1e3,'--','linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$\mathrm{position\, [mm]}$','interpreter','latex','fontsize',18);
legend('$z_U$','$\hat{z}_U$','interpreter','latex','fontsize',14);

figure()
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,3)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
hold on;
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_hat(:,3)+Unscaled_States_Out_hat(:,1))*1e3,'--','linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$\mathrm{position\, [mm]}$','interpreter','latex','fontsize',18);
legend('$z_S$','$\hat{z}_S$','interpreter','latex','fontsize',14);

figure()
plot(Time_Out,Unscaled_Controls_Out,'linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$F\mathrm{\, [N]}$','interpreter','latex','fontsize',18);


figure()
plot(Time_Out,(Unscaled_States_Out_Real(:,3))*1e3,'linewidth',1);
hold on
plot(Time_Out,(Unscaled_States_Out_hat(:,3))*1e3,'--','linewidth',1);
plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),intermediateData.z_vec(:,1).'*1e3,'-o')
xlabel('t [s]','interpreter','latex','fontsize',18);
legend('$z_S-z_U$','$\hat{z}_S-\hat{z}_S$','$z_k$','interpreter','latex','fontsize',18);


figure() %from Ts to tf
plot(Time_Out(auxdata.intermediateData.number_h_over_one_Ts+1:auxdata.intermediateData.number_h_over_one_Ts:end),auxdata.intermediateData.innovation_term(:,output_number),'o-','linewidth',1)
hold on;
plot(Time_Out(auxdata.intermediateData.number_h_over_one_Ts+1:auxdata.intermediateData.number_h_over_one_Ts:end),auxdata.intermediateData.sigma(:,output_number));
plot(Time_Out(auxdata.intermediateData.number_h_over_one_Ts+1:auxdata.intermediateData.number_h_over_one_Ts:end),-auxdata.intermediateData.sigma(:,output_number));
plot(Time_Out(auxdata.intermediateData.number_h_over_one_Ts+1:auxdata.intermediateData.number_h_over_one_Ts:end),2*auxdata.intermediateData.sigma(:,output_number));
plot(Time_Out(auxdata.intermediateData.number_h_over_one_Ts+1:auxdata.intermediateData.number_h_over_one_Ts:end),-2*auxdata.intermediateData.sigma(:,output_number));
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$y-\hat{y},\,\mathrm{Innovation\,\,Term}$','interpreter','latex','fontsize',18);
legend('$y-\hat{y}$','$\sigma_1$','$-\sigma_1$','$2\sigma_1$','$-2\sigma_1$','interpreter','latex','fontsize',14);

figure()
Estimation_Error=y_real(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end)-y_hat(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end);
Measurement_Error=y_real(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end)-intermediateData.z_vec(:,output_number).';
plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),Estimation_Error,'linewidth',1)
hold on
plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),Measurement_Error,'linewidth',1)
xlabel('t [s]','interpreter','latex','fontsize',18);
legend('$\mathrm{Estimation\,\, Error}$','$\mathrm{Measurement\,\,Error}$','interpreter','latex','fontsize',14);

