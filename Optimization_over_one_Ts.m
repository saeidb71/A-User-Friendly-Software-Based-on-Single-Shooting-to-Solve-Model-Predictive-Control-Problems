function [control_variables,objective_value,exit_flag,optimization_output,scaled_states_over_one_P] = Optimization_over_one_Ts(x0,auxdata)

xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint
scaled_states_over_one_P=[]; %states in prediction horizon over h steps

fun = @objfun; % The objective function, nested below
cfun = @constr; % The constraint function, nested below

lb=auxdata.intermediateData.control_variable_lb*ones(numel(x0),1);
ub=auxdata.intermediateData.control_variable_ub*ones(numel(x0),1);

% gs = GlobalSearch;
%     problem = createOptimProblem('fmincon','x0',x0,...
%     'objective',fun,'lb',lb,'ub',ub,'nonlcon',cfun,'options',auxdata.fmincon_option);
% [x,f,eflag,outpt] = run(gs,problem);

%Call fmincon
%tic
[control_variables,objective_value,exit_flag,optimization_output] = fmincon(fun,x0,[],[],[],[],lb,ub,cfun,auxdata.RunMPC.fmincon_option);
%toc

% Ts=auxdata.PMC_params.Ts;
% p=auxdata.PMC_params.p;


% t_control=auxdata.PMC_params.time_control;
% t_final=auxdata.t_final;
% t_control_null_index=[];
% if max(t_control)>t_final
%     t_control_length=numel(t_control);
%     for i=1:t_control_length
%         if(t_control(i)>t_final)
%             t_control_null_index=[t_control_null_index;i];
%         end
%     end
%     t_control(t_control_null_index)=[];
% end
% if numel(t_control)==2
%     t_control=[t_control(1) mean(t_control) t_control(2)];
% end
% nlrhs=zeros(22*(numel(t_control)-1),1);
% %nlrhs=zeros(22*round(p/Ts),1);
% opts=optiset('solver','ipopt','display','iter','derivCheck','on');
% Opt = opti('fun',fun,'lb',lb,'ub',ub,'x0',x0,'nlcon',cfun,'nlrhs',nlrhs,'options',opts);
% [x,f,eflag,outpt] = solve(Opt,x0);

    function y = objfun(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq,scaled_states_over_one_P] = computeall(x,auxdata);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = constr(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq,scaled_states_over_one_P] = computeall(x,auxdata);
            xLast = x;
        end
        % Now compute constraint function
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

end