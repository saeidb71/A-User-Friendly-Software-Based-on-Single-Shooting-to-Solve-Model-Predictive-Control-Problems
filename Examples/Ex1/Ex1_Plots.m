clc;
clear;
close all;

%%
Case1='Mpc  , m: 100  , p: 100  , Ts: 0.01  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case2='Mpc  , m: 20  , p: 20  , Ts: 0.05  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case3='Mpc  , m: 10  , p: 10  , Ts: 0.1  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Case4='Exact_Result.mat';
%Case2='Mpc  , m: 10  , p: 20  , Ts: 0.05  , h: 0.01  , h_const: 0.01  , solver: 2  .mat';
Colors = linspecer(4,'qualitative'); 

figure()
load(Case1);
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1,'color',Colors(1,:),'LineStyle','-');
hold on;
load(Case2);
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1,'color',Colors(2,:),'LineStyle','-');
load(Case3);
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1,'color',Colors(3,:),'LineStyle','-');
load(Case4);
plot(time_finemesh,x_star(:,1),'linewidth',1,'color',Colors(4,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$\xi_1$','interpreter','latex','fontsize',28);
legend('$T_s=0.01\,,\, p=100\,,\, m=100$',...
       '$T_s=0.05\,,\, p=20\,,\, m=20$',...
       '$T_s=0.1\,,\, p=10\,,\, m=10$',...
       '$\mathrm{Exact}$',...
       'interpreter','latex','fontsize',15);
   
figure()
load(Case1);
plot(Time_Out,Unscaled_States_Out_Real(:,2),'linewidth',1,'color',Colors(1,:),'LineStyle','-');
hold on;
load(Case2);
plot(Time_Out,Unscaled_States_Out_Real(:,2),'linewidth',1,'color',Colors(2,:),'LineStyle','-');
load(Case3);
plot(Time_Out,Unscaled_States_Out_Real(:,2),'linewidth',1,'color',Colors(3,:),'LineStyle','-');
load(Case4);
plot(time_finemesh,x_star(:,2),'linewidth',1,'color',Colors(4,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$\xi_2$','interpreter','latex','fontsize',28);
legend('$T_s=0.01\,,\, p=100\,,\, m=100$',...
       '$T_s=0.05\,,\, p=20\,,\, m=20$',...
       '$T_s=0.1\,,\, p=10\,,\, m=10$',...
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
time_finemesh=[0:PMC_params.h/10:tf].';
U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1,'color',Colors(3,:),'LineStyle','-');
load(Case4);
plot(time_finemesh,u_star_val,'linewidth',1,'color',Colors(4,:),'LineStyle','-');
xlabel('$t$','interpreter','latex','fontsize',18);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('$T_s=0.01\,,\, p=100\,,\, m=100$',...
       '$T_s=0.05\,,\, p=20\,,\, m=20$',...
       '$T_s=0.1\,,\, p=10\,,\, m=10$',...
       '$\mathrm{Exact}$',...
       'interpreter','latex','fontsize',15);
   
   
 %% close all
Colors = linspecer(6,'qualitative'); 
load('MPC_m2_p3_Ts0.2_show_states_in_iters.mat');

figure()
hold on;
for iter=1:intermediateData.num_total_iterations
    plot(intermediateData.time_cell_iters_h{iter},intermediateData.scaled_control_one_P_in_h{iter},'--','linewidth',4,'color',Colors(iter,:));
end
plot(Time_Out,intermediateData.scaled_control_matrix,'-','linewidth',1.5,'color',Colors(iter+1,:));
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$u$','interpreter','latex','fontsize',28);
legend('$\mathrm{iter}=1,\,\, \mathrm{time}=[0,\,0.6]$',...
       '$\mathrm{iter}=2,\,\, \mathrm{time}=[0.2,\,0.8]$',...
       '$\mathrm{iter}=3,\,\, \mathrm{time}=[0.4,\,1.0]$',...
       '$\mathrm{iter}=4,\,\, \mathrm{time}=[0.6,\,1.0]$',...
       '$\mathrm{iter}=5,\,\, \mathrm{time}=[0.8,\,1.0]$',...
       '$\mathrm{Correct}$',...
       'interpreter','latex','fontsize',15);
   
figure()
hold on;
for iter=1:intermediateData.num_total_iterations
    plot(intermediateData.time_cell_iters_h{iter},intermediateData.scaled_states_hat_iters_one_P_in_h{iter}(:,1),'--','linewidth',4,'color',Colors(iter,:));
end
plot(Time_Out,intermediateData.scaled_states_matrix_hat(:,1),'-','linewidth',1.5,'color',Colors(iter+1,:));
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$\xi_1$','interpreter','latex','fontsize',28);
legend('$\mathrm{iter}=1,\,\, \mathrm{time}=[0,\,0.6]$',...
       '$\mathrm{iter}=2,\,\, \mathrm{time}=[0.2,\,0.8]$',...
       '$\mathrm{iter}=3,\,\, \mathrm{time}=[0.4,\,1.0]$',...
       '$\mathrm{iter}=4,\,\, \mathrm{time}=[0.6,\,1.0]$',...
       '$\mathrm{iter}=5,\,\, \mathrm{time}=[0.8,\,1.0]$',...
       '$\mathrm{Correct}$',...
       'interpreter','latex','fontsize',15);   
   
figure()
hold on;
for iter=1:intermediateData.num_total_iterations
    plot(intermediateData.time_cell_iters_h{iter},intermediateData.scaled_states_hat_iters_one_P_in_h{iter}(:,2),'--','linewidth',4,'color',Colors(iter,:));
end
plot(Time_Out,intermediateData.scaled_states_matrix_hat(:,2),'-','linewidth',1.5,'color',Colors(iter+1,:));
xlabel('$t$','interpreter','latex','fontsize',28);
ylabel('$\xi_2$','interpreter','latex','fontsize',28);
legend('$\mathrm{iter}=1,\,\, \mathrm{time}=[0,\,0.6]$',...
       '$\mathrm{iter}=2,\,\, \mathrm{time}=[0.2,\,0.8]$',...
       '$\mathrm{iter}=3,\,\, \mathrm{time}=[0.4,\,1.0]$',...
       '$\mathrm{iter}=4,\,\, \mathrm{time}=[0.6,\,1.0]$',...
       '$\mathrm{iter}=5,\,\, \mathrm{time}=[0.8,\,1.0]$',...
       '$\mathrm{Correct}$',...
       'interpreter','latex','fontsize',15);     
   