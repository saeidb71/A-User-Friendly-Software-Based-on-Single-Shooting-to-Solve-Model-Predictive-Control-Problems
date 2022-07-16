function [P]=P_Kalman_Solver(time_cell_iters_h_one_Ts,P0,auxdata,type_num_diff)

h=auxdata.PMC_params.h;
P=zeros(numel(time_cell_iters_h_one_Ts),numel(P0));
P(1,:)=P0;
Kalman_P_func=auxdata.Kalman.P_dynamics;

switch type_num_diff
    case 1
        for i=2:numel(time_cell_iters_h_one_Ts)
            P(i,:)=P(i-1,:)+h*Kalman_P_func(time_cell_iters_h_one_Ts(i-1),P(i-1,:).',auxdata).';
        end
    case 2
        for i=2:numel(time_cell_iters_h_one_Ts)
            k1=Kalman_P_func(time_cell_iters_h_one_Ts(i-1),P(i-1,:).',auxdata);
            k2=Kalman_P_func((time_cell_iters_h_one_Ts(i-1)+time_cell_iters_h_one_Ts(i))/2,P(i-1,:).'+h*k1/2,auxdata);
            k3=Kalman_P_func((time_cell_iters_h_one_Ts(i-1)+time_cell_iters_h_one_Ts(i))/2,P(i-1,:).'+h*k2/2,auxdata);
            k4=Kalman_P_func(time_cell_iters_h_one_Ts(i),P(i-1,:).'+h*k3,auxdata);
            P(i,:)=P(i-1,:)+h/6*(k1+2*k2+2*k3+k4).';
        end
end

end
