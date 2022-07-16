clc;
clear;
close all;
%% 5MW wind Turbine
Irx =38759228;  %J_rotor
rho_air= 1.225; % air density [kg/m^3]
auxdata.rho_air=rho_air;
omega_max=1.2671; %rad/s %rated omega  1.5114: max omega
auxdata.Irx=Irx;
auxdata.omega_max=omega_max;
auxdata.etha=0.944; %drive train efficiency
R_rotor = 63;  auxdata.R_rotor=R_rotor;  % Rotor radius  

% Cp and Ct as a function of lambda(tip speed rato) and beta (pitch angle)
load('CP_CT_MAT.mat') % (Baseline) obtained by Blade Element Momentum Method

[THETA_P,LAMBDA]=meshgrid(Theta_p,Lambda);
Cpfunc=griddedInterpolant(LAMBDA,THETA_P,CP_Mat,'spline');
Ctfunc=griddedInterpolant(LAMBDA,THETA_P,CT_Mat,'spline');
auxdata.Cpfunc=Cpfunc;
auxdata.Ctfunc=Ctfunc;

load('wind_profile_30.mat');
wind_avg=8;
v=@(t) wind_avg*1/14.4607*interp1(t_interp/4,v_interp,t,'spline');
auxdata.v=v; 
P_max = 5e6;  auxdata.P_max=P_max;

t0 = 0;    % [s] 
tf=100; auxdata.tf=tf;    % finale simulation time [s]

lambda_star=7.6; %tip speed ratio that yields maximum Cp
cp_star=0.4978; %max Cp
t_guess=[linspace(0,tf,360)].';
omega_star=lambda_star*auxdata.v(t_guess)/auxdata.R_rotor; %optimal Omega
k_star=pi*auxdata.rho_air*auxdata.R_rotor^5*cp_star/(2*lambda_star^3);
gen_torque_star=k_star*omega_star.^2; %opmtimal generator torque

integral_min=0;   integral_max=2; %intergral: (P_a)*etha/P_max*tf/tf--> min:0, max:1

%-------------------------------------------------------------------------% 
%----------------------- Setup for Problem Bounds ------------------------% 
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0; 
bounds.phase.initialtime.upper = t0; 
bounds.phase.finaltime.lower = tf; 
bounds.phase.finaltime.upper = tf; 
bounds.phase.initialstate.lower = [0.8*omega_max];
bounds.phase.initialstate.upper = [0.8*omega_max];
bounds.phase.finalstate.lower = 0;
bounds.phase.finalstate.upper = omega_max;
bounds.phase.state.lower = 0;
bounds.phase.state.upper = omega_max;
bounds.phase.control.lower= [0, 0] ;
bounds.phase.control.upper= [4.18e6, 0.6807]; 
bounds.phase.integral.upper= integral_max;
bounds.phase.integral.lower= integral_min;

%-------------------------------------------------------------------------% 
%---------------------- Provide Guess of Solution ------------------------% 
%-------------------------------------------------------------------------%

nt=numel(t_guess);
guess.phase.time        = [t_guess];
guess.phase.state       = [omega_star];
guess.phase.control     = [gen_torque_star,(wind_avg-3)*0.6807/22*ones(nt,1)];
guess.phase.integral    = 2/3*integral_max;

%-------------------------------------------------------------------------% 
%----------Provide Mesh Refinement Method and Initial Mesh ---------------% 
%-------------------------------------------------------------------------%
mesh.method           = 'hp-LiuRao-Legendre';
mesh.tolerance        =1e-5;
mesh.maxiterations    = 200; 
mesh.colpointsmin     = 2; 
mesh.colpointsmax     = 11; 

mesh.phase.colpoints  = [9*ones(1,20)]; 
mesh.phase.fraction   = 1/20*ones(1,20);

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name                               = 'Rigid_GPOPS2';
setup.functions.continuous               = @Rotor_drivetrain_Continuous;
setup.functions.endpoint                 = @Rotor_drivetrain_Endpoint;
setup.displaylevel                       = 2;
setup.bounds                             = bounds;
setup.guess                              = guess;
setup.mesh                               = mesh;
setup.auxdata                            = auxdata;
setup.nlp.solver                         = 'ipopt';
setup.nlp.snoptoptions.tolerance         = 1e-8;
setup.nlp.snoptoptions.maxiterations     = 1500; 
setup.nlp.ipoptoptions.linear_solver     = 'ma57'; 
setup.nlp.ipoptoptions.tolerance         = 1e-8;
setup.nlp.ipoptoptions.maxiterations     = 370; 
setup.derivatives.supplier               = 'sparseCD';%adigator%sparseCD
setup.derivatives.dependencies           = 'sparseNaN';
%setup.derivatives.derivativelevel       = 'second'; 
setup.method                             = 'RPM-integration';%RPM-Differentiation
setup.scales.method                      = 'automatic-bounds';

clear output
%-------------------------------------------------------------------------% 
%---------------------- Solve Problem Using GPOPS2 -----------------------% 
%-------------------------------------------------------------------------%
output = gpops2(setup);

solution_interp=output.result.interpsolution;
t=solution_interp.phase.time;
omega=solution_interp.phase.state;
Tgen=solution_interp.phase.control(:,1);
Beta=solution_interp.phase.control(:,2);
wind=auxdata.v(t);

lambda=R_rotor*omega./wind;
Cp_vec=Cpfunc(lambda,Beta);

clearvars -except t omega Tgen wind