function [P_Kalman] = P_Kalman_compute(auxdata)
time_cell_iters_h=auxdata.intermediateData.time_cell_iters_h{auxdata.RunMPC.iter_number};
time_cell_iters_h_one_Ts=time_cell_iters_h(1:auxdata.intermediateData.number_h_over_one_Ts+1,:);

P_at_initial_time=auxdata.intermediateData.P_Last;

dynamic_solver_method=auxdata.PMC_params.dynamic_solver_method;
[P_Kalman]=P_Kalman_Solver(time_cell_iters_h_one_Ts,P_at_initial_time,auxdata,dynamic_solver_method);
end