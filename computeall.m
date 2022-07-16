function [Obj_value,C,Ceq,X_scaled] = computeall(x,auxdata)

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

states_at_initial_time=auxdata.intermediateData.state_last_hat;

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
[time_constraint,X_scaled_constraint,X_scaled]=Dynamic_hat_Solver(time_cell_iters_h,time_cell_iters_Ts,states_at_initial_time,auxdata,dynamic_solver_method);

Ceq=[];

Vector_X_scaled_constraint=X_scaled_constraint(2:end,1:end-1);% from time after initial for all states except for objective
Vector_X_scaled_constraint=Vector_X_scaled_constraint(:);
C_state_bounds=[ Vector_X_scaled_constraint-1
    -Vector_X_scaled_constraint-1];
C_Path_constraint=auxdata.Dynamics.Path_Constraint_func(time_constraint,X_scaled_constraint,auxdata);

C=[C_state_bounds;C_Path_constraint];

Obj_value=auxdata.Model_data.A_scaled(end)+auxdata.Model_data.B_scaled(end,end)*X_scaled_constraint(end,end);

if isnan(Obj_value) || isinf(Obj_value) %if any error happened
    Obj_value=500;
    C=-1*ones(size(C));
end

end