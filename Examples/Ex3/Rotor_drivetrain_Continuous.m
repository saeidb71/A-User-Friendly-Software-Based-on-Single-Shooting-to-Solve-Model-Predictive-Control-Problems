 function phaseout = Rotor_drivetrain_Continuous(input)

t = input.phase.time; 
q = input.phase.state; 
Control= input.phase.control;

dq=zeros(numel(t),1);

Cp=input.auxdata.Cpfunc;
Ct=input.auxdata.Ctfunc; 
R_rotor=input.auxdata.R_rotor;
Irx=input.auxdata.Irx;
rho_air=input.auxdata.rho_air;
etha=input.auxdata.etha;

wind_speed=input.auxdata.v(t);

lambda=R_rotor*q(:,1)./wind_speed;
tau_a=1/2*rho_air*pi*R_rotor^3*Cp(lambda,Control(:,2))./lambda.*wind_speed.^2; %aerodynamic torque

T_gen=Control(:,1);

dq(:,1)=(tau_a-T_gen)/Irx;

phaseout.dynamics=[dq];
P_a=Cp(lambda,Control(:,2))*0.5*rho_air*pi*R_rotor.^2.*wind_speed.^3;
P_max=input.auxdata.P_max; %[W]
phaseout.integrand=((P_a)-1*(1e-9*T_gen.^2+0*1e8*Control(:,2).^2))*etha/P_max/input.auxdata.tf; %[W/W]
 end


