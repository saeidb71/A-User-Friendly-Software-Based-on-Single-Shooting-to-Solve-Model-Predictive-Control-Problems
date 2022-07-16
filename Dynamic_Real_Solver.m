function [X_scaled]=Dynamic_Real_Solver(time_cell_iters_h,time_cell_iters_Ts,X0_scaled,auxdata,type_num_diff)

h=auxdata.PMC_params.h;
X_scaled=zeros(numel(time_cell_iters_h),auxdata.intermediateData.number_of_states);
X_scaled(1,:)=X0_scaled;

U_gridinterp=auxdata.U_gridinterp;

Dynamic_Real_func=auxdata.Dynamics.Dynamic_Real_func;

switch type_num_diff
    case 1
        % size: controles*1
        U_scaled_iminus_1=zeros(auxdata.intermediateData.number_of_controllers,1);
        for i=2:numel(time_cell_iters_h)
            for enum_controlls=1:auxdata.intermediateData.number_of_controllers
                U_scaled_iminus_1(enum_controlls)=U_gridinterp{enum_controlls}(min(max(time_cell_iters_h(i-1),time_cell_iters_Ts(1)),time_cell_iters_Ts(end)));
            end
            X_scaled(i,:)=X_scaled(i-1,:)+h*Dynamic_Real_func(time_cell_iters_h(i-1),X_scaled(i-1,:).',U_scaled_iminus_1,auxdata).';
        end
    case 2
        % size: controles*1
        U_scaled_i=zeros(auxdata.intermediateData.number_of_controllers,1);
        U_scaled_iminus_1=zeros(auxdata.intermediateData.number_of_controllers,1);
        for i=2:numel(time_cell_iters_h)
            for enum_controlls=1:auxdata.intermediateData.number_of_controllers
                U_scaled_i(enum_controlls)=U_gridinterp{enum_controlls}(min(max(time_cell_iters_h(i),time_cell_iters_Ts(1)),time_cell_iters_Ts(end)));
                U_scaled_iminus_1(enum_controlls)=U_gridinterp{enum_controlls}(min(max(time_cell_iters_h(i-1),time_cell_iters_Ts(1)),time_cell_iters_Ts(end)));
            end
            k1=Dynamic_Real_func(time_cell_iters_h(i-1),X_scaled(i-1,:).',U_scaled_iminus_1,auxdata);
            k2=Dynamic_Real_func((time_cell_iters_h(i-1)+time_cell_iters_h(i))/2,X_scaled(i-1,:).'+h*k1/2,(U_scaled_iminus_1+U_scaled_i)/2,auxdata);
            k3=Dynamic_Real_func((time_cell_iters_h(i-1)+time_cell_iters_h(i))/2,X_scaled(i-1,:).'+h*k2/2,(U_scaled_iminus_1+U_scaled_i)/2,auxdata);
            k4=Dynamic_Real_func(time_cell_iters_h(i),X_scaled(i-1,:).'+h*k3,U_scaled_i,auxdata);
            X_scaled(i,:)=X_scaled(i-1,:)+h/6*(k1+2*k2+2*k3+k4).';
        end
end
end
