clc;
clear all;
close all;

temp = load("savedata.txt");

time = temp(:,3);
q = temp(:,4);
qdot = temp(:,5);
q_cmd = temp(:,6);
qdot_cmd = temp(:,7);
error = temp(:,8);
error_dot = temp(:,9);
torque_cmd = temp(:,10);

figure(1)
subplot(2,1,1)
plot(time,q_cmd,'b','linewidth',2,time,q,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("deg",'fontsize',20);
legend('q cmd','q');
title("Position Plot",'fontsize',20);
axis([0 20 -50 50]);

subplot(2,1,2)
plot(time,error,'b','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("deg",'fontsize',20);
legend('error');
title("Position Error Plot",'fontsize',20);
axis([0 20 -1 1]);

figure(2)
subplot(2,1,1)
plot(time,qdot_cmd,'b','linewidth',2,time,qdot,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("deg/s",'fontsize',20);
legend('qdot cmd','qdot');
title("Velocity Plot",'fontsize',20);
axis([0 20 -100 100]);

subplot(2,1,2)
plot(time,error_dot,'b','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("deg/s",'fontsize',20);
legend('error dot');
title("Velocity Error Plot",'fontsize',20);
axis([0 20 -3 3]);

figure(3)
plot(time,torque_cmd,'b','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("N",'fontsize',20);
legend('torque cmd');
title("Torque Plot",'fontsize',20);
axis([0 20 -30 30]);

figure(4)
plot(error,error_dot,'b.','linewidth',2);
grid on; hold on;
plot([-0.1,0.1],[3,-3],'r','linewidth',2);
xlabel("error",'fontsize',20);
ylabel("error dot",'fontsize',20);
axis([-5 5 -100 10]);