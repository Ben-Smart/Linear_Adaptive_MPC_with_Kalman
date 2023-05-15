
function [STATES, Par, endtime] = MPC_loop(Par, ref)
    %time parameters
    Ts = Par.time.Ts; % sampling time
    dt = Par.time.dt; % ODE time step
    Tend = Par.time.Tend; % final time (h)
    Kal = Par.sim.Kal;
    
    %inputs
    INPUT_act = Par.Init_input.INPUT_act; % initial input
    ns = length(Par.sim.x0); % number of states

    
    Cost_val = []; %Cost function value
    X_ERR = [];    %Error values
    

    x_esti = zeros(ns,Tend/Ts+1);% estimated state(initially set to zeros)

    %% initial state of the system
    x_hat = Par.sim.x_hat'; %Current state of the system
    
    x_real = zeros(ns,Tend/Ts+1);% state of the system to be filled
    
    time_dim = ((Tend)/Ts); %number of MPC iterations

    %% initiate the loop for MPC iterations
    
    
        Xref = ref.Xref'; %Create reference vector
    
    j=1; % index for the elements of the reference signal
    Par.sim.j=j;

    %estimate at time j=1
    x_esti(:,j) = x_hat;
    %process state at time j=1
    x_real(:,j) = x_hat;
    tspan = linspace(0,Ts,Ts/dt+1);
    endflag=1;
    
    
%% first step of the controller

 [input_act, cost_val, X_err] = MPC_algorithm(x_hat, Xref, 1,...
                                Par.sim, Par.ctrl,INPUT_act(:,end), Ts, dt);

        Cost_val = [Cost_val; cost_val]; %updated minimum of the cost function
        X_ERR = [X_ERR X_err];           %updated error signal

        
        %if initial input is zero then run this bit, as will obsever and
        %act in the first minute. 
        INPUT_act = input_act;% we need to save it as many times as TC

    %% START POINT OF THE LOOP
    while (endflag)
        %% STATE ESTIMATION
        
        I = [INPUT_act(1,end) INPUT_act(2,end)]'; %current input 
                                                                   % to the system
        Z = [x_hat(:,end);I]; %vector of states and inputs
        
        [~,Z] = ode_solver(Z, tspan); % Plant simulation to the current inputs
        if length(tspan) ==2
            Z = Z([1 end],:);
        end
        x_hat = Z(2:end,1:length(x_hat(:,1)))'; % actual state of the 
                                                % system after the last step
        

         x_real(:,(j-1)*(Ts/dt)+1:(j)*(Ts/dt))=x_hat; %the measured states 
                                                      % throughout the 
                                                      % plant simulation
        if Kal == true
          [x_obs, ~] = Observer(x_hat(1,end),x_esti(:,j),I,dt,Ts); %Kalman filter used as an observer
          x_esti(:,j+1)=x_obs(:,end); %The estimated current plant states
        else
          x_esti(:,j+1)=x_hat(:,end); %The measured current plant states
        end                            
               
        %% optimisation step of future prediction
        
        [input_act, cost_val, X_err] = MPC_algorithm(x_esti(:,j+1),...
                                Xref, j, Par.sim, Par.ctrl,INPUT_act(:,end), Ts, dt);

        Cost_val = [Cost_val; cost_val]; %optimal values of the cost function
        INPUT_act = [INPUT_act input_act];% The input profile
        X_ERR = [X_ERR X_err];% the error between the reference and the current state
        
        %% setting endflag
        endtime = j;
        j=j+1;
        Par.sim.j=j;
        if (j==time_dim)
            endflag=0;
        end
        

    end
    %% OUTPUT VARIABLES

    STATES.x_real = x_real;   % actual state of the plant throughout the simulation
    STATES.cost = Cost_val;   % how the minimum of the cost function varies throughout the simualtion
    STATES.input = INPUT_act; % The input applied to the plant throughout the simulation
    STATES.err = X_ERR;       % The error signal throughout the simulation
  

    end


