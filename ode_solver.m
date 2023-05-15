function [ t,xout ] = ode_solver(Z0, tspan)
[t,xout]=ode23tb(@f,tspan,Z0);
end



function xdot=f(t,x)    

alpha_x = 0;
alpha_y = 0.8;
alpha_0 = 0.9;
alpha_k = 1.7;

k = 0.0001;

beta_x = 0.9;
beta_y = 1.2;
eta = 1;
in = 1;

  xdot=zeros(5,1);

	xdot(1) = beta_x*x(4) - alpha_x*x(1) - alpha_k*x(3)*(x(1)/(x(1) + k)) + in*x(5);

	xdot(2) = beta_y*x(1)*eta - alpha_0*x(2);

    xdot(3) = alpha_0*x(2) - alpha_y*x(3);

end
