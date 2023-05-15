
clear; clc;
%% MPC setup

%% Function Overview: 

%   MPC_setup -     All the parameter settings for the MPC simulation, 
%                   the calculation of the Error/Dose Index and a simple 
%                   plot.  

%   MPC_loop -      initiates the real time loop of MPC, includes the plant 
%                   block of the system through the ode_solver and has been 
%                   left to include the estimator variables, but this has 
%                   not been added yet, but a Kalman filter can be added. 

%   MPC_algorithm - sets up the cost function for all of the predicted
%                   future steps.

%   X0 -            Initial conditions of states.

%   X0_ref -        Initial conditions of reference.

%   Xol  -          The free responce of the system (only used for plotting).

%   Xol_ref  -      The reference used within the controller.

%   State_Space -   the linearisation of the system, add how the linear
%                   model is produced from the state conditions.

%   ode_solver -    acts as the Plant block in the MPC, it is currently 
%                   the non-linear model of the P53 system but could be
%                   the recieved measurments if you are working on an 
%                   actual experiment. Add the system to be acted apon 
%                   under function 'f'.


% The controller is set to follow the reference and currently only has 
% input constraints. 


%% parameters

Par.time.Ts = 0.1;                    % sample time of the controller(h)
Par.time.dt = 0.01;                   % Time step for the ODE solver(h)
Par.time.Tend =50;                    % final runtime (h)

Par.sim.x_hat = X0;                   % the state of the system at time j
Par.sim.x0 = X0;                      % initial state
Par.sim.N = 100;                      % steps in the prediction horizon
Par.sim.n_inputs = 2;                 % the number of inputs (for my example one input is a constant protien production)

Par.Init_input.INPUT_act = [0 1]';    % initial inputs

Par.ctrl.weights.alpha = 0;           % cost on the internal state errors
Par.ctrl.weights.beta = 1;            % extra cost on the outputs
Par.ctrl.weights.gamma = [0.05 0];    % cost on the input
Par.ctrl.Lb = [-1 1];                 % lower bound for the control input
Par.ctrl.Ub = [1 1];                  % upper bound for the control input
Par.ctrl.options = optimoptions...    % options for the quadratic solver 
    ('quadprog','display','none');    % which reduce the runtime of the 
                                      % programe, but will show no warning
                                      % messages.
                                      
%% Signals

[~,xol] = Xol(Par.time,Par.sim.x0);        % Free response - no inputs
[T,ref.Xref] = Xol_ref(Par.time,X0_ref);   % Reference response
                       

%% MPC function

[STATES, Par, endtime] = MPC_loop(Par, ref);


%% plot

x_real = STATES.x_real; % Plant states
INPUT_act = STATES.input; % input profiles
t = linspace(0,length(x_real(1,:))*Par.time.dt,length(x_real(1,:))); %time step of output states
tnp = linspace(0,Par.time.Tend,Par.time.Tend/Par.time.Ts); % time steps of the inputs
set(0, 'DefaultLineLineWidth', 2);

subplot(2,1,1)
plot(T,ref.Xref(:,1),T,xol(:,1),t,x_real(1,:),'--')
legend('Reference','Free Responce','Controlled Output')
  legend('boxoff')
  legend('Orientation','horizontal')
  legend('Location','northwest')
title('Output')
xlabel('Time(Hr)')
axis([0 Par.time.Tend -0.05 1.1])


subplot(2,1,2)
plot(tnp,INPUT_act(1,:),'k')
title('Input')
xlabel('Time(Hr)')
axis([0 Par.time.Tend -0.7 0.01])
 set(findall(gcf,'-property','FontSize'),'FontSize',18)
