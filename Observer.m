function [x_obs, L] = Observer(y,x_kminus,I,dt,Ts)

[Ad,Bd,Cd,Dd,~,~,~,~] = State_Space(x_kminus,I,dt);
% y = Cd*measurment;

%% kalman filter
sys = dss(Ad,[Bd Bd],Cd,[Dd Dd],eye(size(Ad,1)),Ts);%SYS=SS(A,[B G],C,[D H])
Q = 0.5*eye(2);
R = 0.13;
N = [0.03; 0.03];
known = [1 2];
sensors = [1];

[kalmf,L,P] = kalman(sys,Q,R,N,sensors,known);

%% pole placement
% sys = ss(Ad,Bd,Cd,Dd,Ts);
% %pole = linspace(-0.81,0.8,size(Ad,1));
% %pole = [-0.5*ones(1,floor(size(Ad,1)/2)) 0.5*ones(1,ceil(size(Ad,1)/2))];
% %rank(Bd)
% pole_1 = linspace(-0.81,0.8,floor(size(Ad,1)/2));
% pole_2 = linspace(-0.81,0.8,ceil(size(Ad,1)/2));
% pole = zeros(1,size(Ad,1));
% pole(2:2:floor(size(Ad,1)/2)*2) = pole_1;
% pole(1:2:end) = pole_2;
% L = place(Ad',Cd',pole);
%est = estim(sys,L);

% sys1 = ss(Ad,Bd(:,1),Cd(1,:),[],Ts);
% sys2 = ss(Ad,Bd(:,2),Cd(1,:),[],Ts);
% sys3 = ss(Ad,Bd(:,3),Cd(1,:),[],Ts);
% sys4 = ss(Ad,Bd(:,1),Cd(2,:),[],Ts);
% sys5 = ss(Ad,Bd(:,2),Cd(2,:),[],Ts);
% sys6 = ss(Ad,Bd(:,3),Cd(2,:),[],Ts);
% rlocus(sys1)
%rlocus(sys1,sys2,sys3,sys4,sys5,sys6)

%% Gain guess

% L = 1e-2*ones(size(Cd'));
%est = estim(sys,L);

%% observer
x_obs = x_kminus;
flag = 1;
counter = 0;

while (flag)
x_obs = Ad*x_obs + Bd*I + L*(y - Cd*x_obs);
err = y - Cd*x_obs;
counter = counter +1;

if (abs(err) < 1e-7)
    flag = 0;
end

counter_time = counter*dt;

if (counter_time > Ts)
    flag = 0;
end


end


end