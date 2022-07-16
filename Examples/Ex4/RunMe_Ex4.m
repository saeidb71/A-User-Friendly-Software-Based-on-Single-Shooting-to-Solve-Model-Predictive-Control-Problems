clear;
clc;
close all;
delete(gcp('nocreate'));

%% System Data
Model_data.b1=100.8107;
Model_data.k1=2.1521e+04;

% road profile
Model_data.pp.v = 10; % m/s vehicle velocity
load('Road_Profile.mat'); % road profile data
Model_data.road_x=road_x;
Model_data.road_z=road_z;
Model_data.road_t = Model_data.road_x./Model_data.pp.v; % time vector for road profile

Model_data.z0dot = diffxy(Model_data.road_t,Model_data.road_z); % approximate derivative of road velocity
Model_data.pp.z0dot = griddedInterpolant(Model_data.road_t,Model_data.z0dot);
Model_data.pp.z0 = griddedInterpolant(Model_data.road_t,Model_data.road_z);

Model_data.pp.t0 = Model_data.road_t(1); % initial time
Model_data.z0=Model_data.pp.z0;
Model_data.z0dot=Model_data.pp.z0dot;

% fixed suspension parameters
Model_data.bt = 0; % N/(m/s)
Model_data.kt = 232.5e3; % N/m
Model_data.mu = 65; % kg
Model_data.ms = 325; % kg

% rattlespace and stroke constraint
Model_data.rmax = 0.04; % m
Model_data.smax = 0.04; % m

% objective function weights
Model_data.w1 = 1e5;
Model_data.w2 = 0.5;
Model_data.w3 = 1e-5;

%Scaling
% A(i)=(max(state(i))+min(state(i)))/2
Model_data.A_scaled=[0 ; 0; 0; 0; 5];
%B(i,i)=(max(state(i))-min(state(i)))/2
Model_data.B_scaled=diag([10; 10; Model_data.rmax; 10; 5]);
%C(i)=(max(control(i))+min(control(i)))/2
Model_data.C_scaled=[0].';
%D(i,i)=(max(control(i))-min(control(i)))/2
Model_data.D_scaled=[1e3];

Model_data.t0 = 0;    % [s]
Model_data.tf=3.0;    % finale simulation time [s]

Model_data.initial_state_Real=[0 0 0 0 -1].';
Model_data.initial_state_hat=[0 0 0 0 -1].';
%Model_data.initial_state_hat=[-0.01 -0.03 0.7 0.04 -1].';

%% Dynamics
Dynamics.Dynamic_hat_func=@(t,q_scaled_with_obj,U_scaled,auxdata) Dynamic_hat(t,q_scaled_with_obj,U_scaled,auxdata);
Dynamics.Dynamic_Real_func=@(t,q_scaled_with_obj,U_scaled,auxdata) Dynamic_Real(t,q_scaled_with_obj,U_scaled,auxdata);
%Dynamics.Dynamic_Real_func=@(t,q_scaled_with_obj,U_scaled,auxdata) Dynamic_hat(t,q_scaled_with_obj,U_scaled,auxdata);
Dynamics.Path_Constraint_func=@(t,q_scaled,auxdata) Path_constraint(t,q_scaled,auxdata);

%% MPC parameters
PMC_params.Ts=0.1;      % Control Sampling Time (s)
PMC_params.m=2;          % Control Horizon
PMC_params.p=5;          % Prediction Horizon
PMC_params.h=0.01;      % Time Step Size
PMC_params.constraint_step_time=max(0.01,PMC_params.h);
PMC_params.dynamic_solver_method=2; %2 : Runge Kuttah 4th order , 1: Euler
PMC_params.x0_init=[0].';
%% fmincon parameters
fmincon_options.number_of_cores=1;
if fmincon_options.number_of_cores>1
    parpool(fmincon_options.number_of_cores);
    fmincon_options.parallel_flag=true;
else
    fmincon_options.parallel_flag=false;
end
fmincon_options.optimoptions=optimoptions(@fmincon,'Algorithm','interior-point','display','iter','TolCon',1e-9,'TolFun',1e-9,'MaxFunEvals',1e7,'StepTolerance',1e-6,'MaxIter',100,'UseParallel',fmincon_options.parallel_flag);

%% Kalman Filter
Kalman.flag=true;
if Kalman.flag
    Kalman.QR_mat='Kalman_Matrices.mat';
    load(Kalman.QR_mat)
    Kalman.Q_Kalman=Q_Kalman*0.01;
    Kalman.R_Kalman=R_Kalman;
    Kalman.P0_Kalman=10*P0_Kalman(:).';
    Kalman.H_Kalman=[0 0 1 0]; % y=H*x
    %Kalman.H_Kalman=eye(4); % y=H*x
    Kalman.P_dynamics=@(t,q,auxdata) Kalman_P_func(t,q,auxdata);
else
    Kalman.Q_Kalman=zeros(numel(Model_data.A_scaled)-1,numel(Model_data.A_scaled)-1);
end

%% Run MPC
tic
[Time_Out,Unscaled_States_Out_Real,Unscaled_States_Out_hat,Unscaled_Controls_Out,auxdata] = RunMPC_func(Model_data,Dynamics,PMC_params,Kalman,fmincon_options);
toc
Model_data=auxdata.Model_data;
Dynamics=auxdata.Dynamics;
PMC_params=auxdata.PMC_params;
intermediateData=auxdata.intermediateData;

saved_workspace_name = ['Kalman_Mpc'...
                        '  , m: ' num2str(PMC_params.m)...
                        '  , p: ' num2str(PMC_params.p)...
                        '  , Ts: ' num2str(PMC_params.Ts)...
                        '  , h: ' num2str(PMC_params.h)...
                        '  , h_const: ' num2str(PMC_params.constraint_step_time)...
                        '  , solver: ' num2str(PMC_params.dynamic_solver_method)...
                        '  .mat'];
save(saved_workspace_name);

%% unscaled
figure()
plot(Time_Out,Model_data.pp.z0(Time_Out)*1e3,'linewidth',1);
hold on
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
plot(Time_Out,(Model_data.pp.z0(Time_Out)+Unscaled_States_Out_Real(:,3)+Unscaled_States_Out_Real(:,1))*1e3,'linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$\mathrm{position\, [mm]}$','interpreter','latex','fontsize',18);
legend('$\delta$','$z_U$','$z_S$','interpreter','latex','fontsize',14);

figure()
plot(Time_Out,Unscaled_Controls_Out,'linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$F\mathrm{\, [N]}$','interpreter','latex','fontsize',18);

figure()
plot(Time_Out,abs(Unscaled_States_Out_Real(:,3))*1e3,'linewidth',1);
hold on
plot(Time_Out,abs(Unscaled_States_Out_Real(:,3))*1e3,'--','linewidth',1);
xlabel('t [s]','interpreter','latex','fontsize',18);
ylabel('$|r|\, , S_{\mathrm{max}}\,\mathrm{[mm]}$','interpreter','latex','fontsize',18);
legend('$|r|$','$S_{\mathrm{max}}$','interpreter','latex','fontsize',14);

if Kalman.flag
    output_number=1;
    
    figure()
    plot(Time_Out,(Unscaled_States_Out_Real(:,3))*1e3,'linewidth',1);
    hold on
    plot(Time_Out,(Unscaled_States_Out_hat(:,3))*1e3,'--','linewidth',1);
    plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),intermediateData.z_vec(:,1).'*1e3,'-o')
    xlabel('t [s]','interpreter','latex','fontsize',18);
    legend('$x_3$','$\hat{x}_3$','$z_k$','interpreter','latex','fontsize',18);

    figure()
    plot(Time_Out,intermediateData.Estimation_Error_var(:,1),'linewidth',1)
    xlabel('t [s]','interpreter','latex','fontsize',18);
    ylabel('$\mathrm{Estimaiton\,\, error\,\, variance}$','interpreter','latex','fontsize',18);

    
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

    y_real=Kalman.H_Kalman*Unscaled_States_Out_Real(:,1:numel(Model_data.A_scaled)-1).';
    y_hat=Kalman.H_Kalman*Unscaled_States_Out_hat(:,1:numel(Model_data.A_scaled)-1).';
    
    
    figure()
    Estimation_Error=y_real(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end)-y_hat(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end);
    Measurement_Error=y_real(output_number,1:auxdata.intermediateData.number_h_over_one_Ts:end)-intermediateData.z_vec(:,output_number).';
    plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),Estimation_Error,'linewidth',1)
    hold on
    plot(Time_Out(1:auxdata.intermediateData.number_h_over_one_Ts:end),Measurement_Error,'linewidth',1)
    xlabel('t [s]','interpreter','latex','fontsize',18);
    legend('$\mathrm{Estimation\,\, Error}$','$\mathrm{Measurement\,\,Error}$','interpreter','latex','fontsize',14);

end

total_obj=Unscaled_States_Out_Real(end,end);
Objective_MPC=total_obj;


plot(Time_Out,intermediateData.scaled_control_matrix,'-','linewidth',2);
hold on;
for iter=1:intermediateData.num_total_iterations
    plot(intermediateData.time_cell_iters_h{iter},intermediateData.scaled_control_one_P_in_h{iter},'--','linewidth',2);

end

