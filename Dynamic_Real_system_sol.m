function [time_one_Ts_based_on_h,X0_Last_one_Ts_based_on_h] = Dynamic_Real_system_sol(x,auxdata)

% size: number_of_controls*number_of_variables
u_index_variables=zeros(auxdata.intermediateData.number_of_controllers,auxdata.intermediateData.m_cell_iters{auxdata.RunMPC.iter_number});

for enum_controlls=1:auxdata.intermediateData.number_of_controllers
    u_index_variables(enum_controlls,:)=auxdata.intermediateData.u_cell_iters{auxdata.RunMPC.iter_number,enum_controlls};
end

% size: number_of_controls*number_of_variables
u_variables=zeros(auxdata.intermediateData.number_of_controllers,auxdata.intermediateData.m_cell_iters{auxdata.RunMPC.iter_number});

for enum_controlls=1:auxdata.intermediateData.number_of_controllers
    u_variables(enum_controlls,:)=x(u_index_variables(enum_controlls,:));
end

time_cell_iters_Ts=auxdata.intermediateData.time_cell_iters_Ts{auxdata.RunMPC.iter_number};
time_cell_iters_h=auxdata.intermediateData.time_cell_iters_h{auxdata.RunMPC.iter_number};

states_at_initial_time=auxdata.intermediateData.state_last_Real;

%size: number_of_controls*number_of_variables
u_variables_extended_over_P=zeros(auxdata.intermediateData.number_of_controllers,numel(auxdata.intermediateData.time_cell_iters_Ts{auxdata.RunMPC.iter_number}));

for enum_controlls=1:auxdata.intermediateData.number_of_controllers
    u_variables_extended_over_P(enum_controlls,:)=[u_variables(enum_controlls,:),u_variables(enum_controlls,end)*...
    ones(1,numel(time_cell_iters_Ts)-numel(u_variables(enum_controlls,:)))];
end

U_gridinterp=cell(auxdata.intermediateData.number_of_controllers,1);
for enum_controlls=1:auxdata.intermediateData.number_of_controllers
    U_gridinterp{enum_controlls}=griddedInterpolant(time_cell_iters_Ts,u_variables_extended_over_P(enum_controlls,:),'previous');
end
auxdata.U_gridinterp=U_gridinterp;

dynamic_solver_method=auxdata.PMC_params.dynamic_solver_method;
[X_scaled]=Dynamic_Real_Solver(time_cell_iters_h(1:auxdata.intermediateData.number_h_over_one_Ts+1,:),time_cell_iters_Ts(1:2),states_at_initial_time,auxdata,dynamic_solver_method);

time_one_Ts_based_on_h=time_cell_iters_h(1:auxdata.intermediateData.number_h_over_one_Ts+1,:);
X0_Last_one_Ts_based_on_h=X_scaled;
end