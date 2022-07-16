clc;
clear;
close all;

%%
Case1='Mpc  , m: 50  , p: 50  , Ts: 0.02  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case2='Mpc  , m: 10  , p: 10  , Ts: 0.02  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case3='Exact_Result.mat';
Colors = linspecer(3,'qualitative'); 

figure()
load(Case1);
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1,'color',Colors(1,:),'LineStyle','-');
hold on;
load(Case2);
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1,'color',Colors(2,:),'LineStyle','-');
load(Case3);
plot(time_finemesh,x_star(:,1),'linewidth',1,'color',Colors(3,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$\xi_1$','interpreter','latex','fontsize',28);
legend('$T_s=0.02\,,\, p=50\,,\, m=50$',...
       '$T_s=0.02\,,\, p=10\,,\, m=10$',...
       '$\mathrm{Exact}$',...
       'interpreter','latex','fontsize',15);
   
figure()
load(Case1);
time_finemesh=[0:PMC_params.h/10:tf].';
U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1,'color',Colors(1,:),'LineStyle','-');
hold on
load(Case2);
time_finemesh=[0:PMC_params.h/10:tf].';
U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1,'color',Colors(2,:),'LineStyle','-');
load(Case3);
plot(time_finemesh,u_star_val,'linewidth',1,'color',Colors(3,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',18);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('$T_s=0.02\,,\, p=50\,,\, m=50$',...
       '$T_s=0.02\,,\, p=10\,,\, m=10$',...
       '$\mathrm{Exact}$',...
       'interpreter','latex','fontsize',15);