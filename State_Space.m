               %% State_Space 
% the linearisation of the system - how the linear model is produced
% from the state conditions. 

function [Ad,Bd,Cd,Dd,A,B,C,D] = State_Space(x,Iss,Ts)
%% Parameters in the model

alpha_y = 0.8;
alpha_0 = 0.9;
alpha_k = 1.7;
k = 0.0001;
beta_x = 0.9;
beta_y = 1.2;

%input parameter
in = 1;

    %% %% %% State space formulation %% %% %%
    
%% A matrix
A = zeros(3,3);

[A(1,3),A(1,1)] = hill([alpha_k,1,k],x(3),x(1),1);

A(2,1) = beta_y;
A(2,2) = -alpha_0;

A(3,2) = alpha_0;
A(3,3) = -alpha_y;

%% B matrix

B = [in, beta_x;
    0,0;
    0,0];

%% C matrix

C = zeros(1,3);

C(1,1) = 1; 

%% D matrix

D=[];

%% create a state space of the system

sys = ss(A,B,C,D);

%% descreatised with Zero Order Hold


sysd1 = c2d(sys,Ts,'zoh');

Ad = sysd1.A;

Bd = sysd1.B;

Cd = sysd1.C;

Dd = sysd1.D;

end

%% derivative of the hills terms for the state space

function [Rx,Ry] = hill(p,x,y,m) 

k = p(1);
n = p(2);
KM = p(3);

Rx = (k*y^n)/(KM^n + y^n);
Ry = (k*n*x*(y^(n-1))*KM^n)/(KM^n + y^n)^2;

if m ==1
    Rx = -Rx;
    Ry = -Ry;
end

end