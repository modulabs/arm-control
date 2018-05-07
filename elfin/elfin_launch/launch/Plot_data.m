Sampling = 0.001;

temp = load("Joint2.txt");

time = temp(:,3)*Sampling;
desired_pos = temp(:,4);
current_pos = temp(:,5);
desired_vel = temp(:,6);
current_vel = temp(:,7);
desired_torque = temp(:,8);
error_pos = temp(:,9);
error_vel = temp(:,10);

figure(1)
plot(time,desired_pos,'b',time,current_pos,'r','linewidth',2);
grid on;
xlabel("time(s)");
ylabel("deg");
title("Position");

figure(2)
plot(time,desired_vel,'b','linewidth',2,time,current_vel,'r','linewidth',2);
grid on;
xlabel("time(s)");
ylabel("deg/s");
title("Velocity");

figure(3)
plot(time,desired_torque,'b','linewidth',2);
grid on;
xlabel("time(s)");
ylabel("Nm");
title("Torque");

figure(4)
plot(time,error_pos,'b','linewidth',2);
grid on;
xlabel("time(s)");
ylabel("deg");
title("ErrorPos");

figure(5)
plot(time,error_vel,'b','linewidth',2);
grid on;
xlabel("time(s)");
ylabel("deg/s");
title("ErrorVel");