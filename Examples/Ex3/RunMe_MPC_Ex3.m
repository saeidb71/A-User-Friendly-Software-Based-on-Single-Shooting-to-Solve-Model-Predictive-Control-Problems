clear;
clc;
close all;
delete(gcp('nocreate'));

%% System Data

a=1;
b=@(t) t.*cos(20*pi*t)-1/4;
Model_data.a=a;
Model_data.b=b;

if a==1
    x_tf_star=0.813517;
elseif a==2
    x_tf_star=0.649528;
end

Model_data.x_tf_star=x_tf_star;

tf=1;

Model_data.t0 = 0;    % [s]
Model_data.tf=tf;    % finale simulation time [s]

%Scaling
% A(i)=(max(state(i))+min(state(i)))/2
Model_data.A_scaled=[0;5];
%B(i,i)=(max(state(i))-min(state(i)))/2
Model_data.B_scaled=diag([2;5]);
%C(i)=(max(control(i))+min(control(i)))/2
Model_data.C_scaled=[0].';
%D(i,i)=(max(control(i))-min(control(i)))/2
Model_data.D_scaled=diag([1]);

Model_data.initial_state_Real=[((1)-Model_data.A_scaled(1))/Model_data.B_scaled(1,1) -1].';
Model_data.initial_state_hat=[((1)-Model_data.A_scaled(1))/Model_data.B_scaled(1,1) -1].';

%% Dynamics
Dynamics.Dynamic_hat_func=@(t,q_scaled_with_obj,U_scaled,auxdata) Dynamic_hat(t,q_scaled_with_obj,U_scaled,auxdata);
Dynamics.Dynamic_Real_func=@(t,q_scaled_with_obj,U_scaled,auxdata) Dynamic_Real(t,q_scaled_with_obj,U_scaled,auxdata);
Dynamics.Path_Constraint_func=@(t,q_scaled,auxdata) Path_constraint(t,q_scaled,auxdata);

%% MPC parameters
PMC_params.Ts=0.02;      % Control Sampling Time (s)
PMC_params.m=10;          % Control Horizon
PMC_params.p=10;          % Prediction Horizon
PMC_params.h=0.01;      % Time Step Size
PMC_params.constraint_step_time=max(0.01,PMC_params.h);
PMC_params.dynamic_solver_method=2; %2 : Runge Kuttah 4th order , 1: Euler
PMC_params.x0_init=[0].';

%% fmincon parameters
fmincon_options.number_of_cores=4;
if fmincon_options.number_of_cores>1
    parpool(fmincon_options.number_of_cores);
    fmincon_options.parallel_flag=true;
else
    fmincon_options.parallel_flag=false;
end
fmincon_options.optimoptions=optimoptions(@fmincon,'Algorithm','interior-point','display','iter','TolCon',1e-9,'TolFun',1e-9,'MaxFunEvals',1e7,'StepTolerance',1e-6,'MaxIter',1000,'UseParallel',fmincon_options.parallel_flag);

%% Kalman Filter
Kalman.flag=false;

%% Run MPC
tic
[Time_Out,Unscaled_States_Out_Real,Unscaled_States_Out_hat,Unscaled_Controls_Out,auxdata] = RunMPC_func(Model_data,Dynamics,PMC_params,Kalman,fmincon_options);
toc
Model_data=auxdata.Model_data;
Dynamics=auxdata.Dynamics;
PMC_params=auxdata.PMC_params;
intermediateData=auxdata.intermediateData;

saved_workspace_name = ['Mpc'...
                        '  , m: ' num2str(PMC_params.m)...
                        '  , p: ' num2str(PMC_params.p)...
                        '  , Ts: ' num2str(PMC_params.Ts)...
                        '  , h: ' num2str(PMC_params.h)...
                        '  , h_const: ' num2str(PMC_params.constraint_step_time)...
                        '  , solver: ' num2str(PMC_params.dynamic_solver_method)...
                        '  .mat'];
save(saved_workspace_name);

%% unscaled
total_obj=Unscaled_States_Out_Real(end,end);
Objective_MPC=total_obj;

time_finemesh=[0:PMC_params.h/10:tf].';

U_MPC_finemesh=griddedInterpolant(Time_Out,Unscaled_Controls_Out(:,1),'previous');


u_star_val=zeros(numel(time_finemesh),1);
for time_index=1:numel(time_finemesh)
    u_star_val(time_index)=u_star_func(time_finemesh(time_index),Model_data);
end

odefun=@(t,x) dynamic(t,x,Model_data);

[t_star,x_star] = ode45(odefun,time_finemesh,[1].');

figure()
plot(Time_Out,Unscaled_States_Out_Real(:,1),'linewidth',1);
hold on
plot(time_finemesh,x_star(:,1),'linewidth',1);
hold on
xlabel('$t$','interpreter','latex','fontsize',18);
ylabel('$x_1$','interpreter','latex','fontsize',18);
legend('MPC','Exact','interpreter','latex','fontsize',18);

figure()
plot(time_finemesh,U_MPC_finemesh(time_finemesh),'linewidth',1);
hold on
plot(time_finemesh,u_star_val,'linewidth',1);
xlabel('$t$','interpreter','latex','fontsize',18);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('MPC','Exact','interpreter','latex','fontsize',18);
