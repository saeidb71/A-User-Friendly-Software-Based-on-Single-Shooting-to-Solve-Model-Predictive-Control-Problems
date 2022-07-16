clc;
clear;
close all;

%%
Case1='Mpc  , m: 50  , p: 50  , Ts: 0.01  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case2='Mpc  , m: 10  , p: 10  , Ts: 0.05  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case3='OL_Result.mat';
Colors = linspecer(3,'qualitative'); 

% figure()
% load(Case1);
% plot(Time_Out,Model_data.pp.z0(Time_Out)*1e3,'linewidth',1);
% hold on
% load(Case2);
% plot(Time_Out,Model_data.pp.z0(Time_Out)*1e3,'linewidth',1);
% load(Case3);
% plot(t,p.z0(t)*1e3,'linewidth',1);
% xlabel('t [s]','interpreter','latex','fontsize',18);
% ylabel('$\delta\,\mathrm{[mm]}$','interpreter','latex','fontsize',18);
% legend('$T_s=0.01\,,\, p=50\,,\, m=50$',...
%        '$\mathrm{Exact}$',...
%        'interpreter','latex','fontsize',15);
   

% figure()
% load(Case1);
% plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
% hold on
% load(Case2);
% plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
% load(Case3);
% plot(t,(p.z0(t)+x1)*1e3,'linewidth',1);
% xlabel('t [s]','interpreter','latex','fontsize',18);
% ylabel('$z_U\,\mathrm{[mm]}$','interpreter','latex','fontsize',18);
% legend('$T_s=0.01\,,\, p=50\,,\, m=50$',...
%        '$\mathrm{Exact}$',...
%        'interpreter','latex','fontsize',15);
   
% figure()
% load(Case1);
% plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,3)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
% hold on
% load(Case2);
% plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,3)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
% load(Case3);
% plot(t,(p.z0(t)+x3+x1)*1e3,'linewidth',1);
% xlabel('t [s]','interpreter','latex','fontsize',18);
% ylabel('$z_S\,\mathrm{[mm]}$','interpreter','latex','fontsize',18);
% legend('$T_s=0.01\,,\, p=50\,,\, m=50$',...
%        '$\mathrm{Exact}$',...
%        'interpreter','latex','fontsize',15);   
   
figure()
load(Case1);
time_finemesh=[0:PMC_params.h/10:Model_data.tf].';
U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1,'color',Colors(1,:),'LineStyle','-');
hold on
load(Case2);
time_finemesh=[0:PMC_params.h/10:Model_data.tf].';
U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1,'color',Colors(2,:),'LineStyle','-');
load(Case3);
plot(t,u,'linewidth',1,'color',Colors(3,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',18);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('$T_s=0.01\,,\, p=50\,,\, m=50$',...
       '$T_s=0.05\,,\, p=10\,,\, m=10$',...
       '$\mathrm{Exact}$',...
       'interpreter','latex','fontsize',15);