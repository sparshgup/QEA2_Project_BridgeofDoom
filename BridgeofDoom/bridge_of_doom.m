clc, clear

syms t

%assumptions speed up the computation
assume(t,'real');
assume(t,'positive');

%define parametric function
ri=(0.3960 * cos(2.65 * (0.1 * t + 1.4)));
rj=(-0.99 * sin(0.1 * t + 1.4));
rk=0 * t;
r = [ri, rj, rk];

%calculating linear velocity
drdt = diff(r,t);
linvel = norm(drdt);

%calculating angular velocity
T_hat=drdt/norm(drdt);
dT_hatdt=diff(T_hat,t);
angvel=cross(T_hat,dT_hatdt);
Angular_Velocity=angvel(3);

%calculating left and right wheel speeds
vleft = linvel - (Angular_Velocity * (0.245/2));
vright = linvel + (Angular_Velocity * (0.245/2));

%initialising neato function parameters

drivetime=29; %golden time

%Connect to your Neato
[sensors,vels] = neato('192.168.16.100'); 

%pause(10) %wait a bit for the robot to start up if using the physical neato

tic %%start your timer in Matlab
s=toc; %initiate t as the time since you started
while s<drivetime
    s=toc; %t update t
    vels.lrWheelVelocitiesInMetersPerSecond=[double(subs(vleft,t,s)),double(subs(vright,t,s))]; 
    pause(.01) %you can add a short delay to be safe/for communication lag in the physical neato. 
end

vels.lrWheelVelocitiesInMetersPerSecond=[0,0];
pause(1);
