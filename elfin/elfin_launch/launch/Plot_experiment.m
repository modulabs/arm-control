clc;
clear all;
close all;

temp = load("savedata.txt");

time = temp(:,3);
Xe = temp(:,4);
Xc = temp(:,5);
desired_force = temp(:,6);
actual_force_filt = temp(:,7);
actual_force_nofilt = temp(:,8);
b_gain = temp(:,9);

figure(1)
subplot(2,1,1)
plot(time,Xe,'b','linewidth',2,time,Xc,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("m",'fontsize',20);
legend('Xe','Xc');
title("Position Plot",'fontsize',20);
axis([17 47 0.1 0.2]);
set(gca,'fontsize',10);

subplot(2,1,2)
plot(time,desired_force,'b','linewidth',2,time,actual_force_filt,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("N",'fontsize',20);
legend('DesiredForce','ActualForce');
title("Force Plot",'fontsize',20);
axis([17 47 -1 45]);
set(gca,'fontsize',10);

figure(2)
plot(time,actual_force_nofilt,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
ylabel("N",'fontsize',20);
legend('ActualForceNofilt');
title("Force Plot",'fontsize',20);
axis([17 47]);
set(gca,'fontsize',10);

figure(3)
plot(time,b_gain,'r','linewidth',2);
grid on;
xlabel("time(s)",'fontsize',20);
legend('B Gain');
title("B Gain Plot",'fontsize',20);
axis([17 47]);
set(gca,'fontsize',10);
