function plotJointEvolution(Motion)
%% Plot the joints positions
figure(1)

subplot(2,3,1)
plot(Motion.time,Motion.q(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q1(rad)')

subplot(2,3,2)
plot(Motion.time,Motion.q(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q2(rad)')

subplot(2,3,3)
plot(Motion.time,Motion.q(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q3(cm)')

subplot(2,3,4)
plot(Motion.time,Motion.q(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q4(rad)')

subplot(2,3,5)
plot(Motion.time,Motion.q(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q5(rad)')

subplot(2,3,6)
plot(Motion.time,Motion.q(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q6(rad)')

set(gcf,'NextPlot','add');
axes;
h = title('Joints Positions');
set(gca,'Visible','off');
set(h,'Visible','on');

%% Plot the joints velocities
figure(2)

subplot(2,3,1)
plot(Motion.time,Motion.dq(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq1(rad/s)')

subplot(2,3,2)
plot(Motion.time,Motion.dq(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq2(rad/s)')

subplot(2,3,3)
plot(Motion.time,Motion.dq(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq3(cm/s)')

subplot(2,3,4)
plot(Motion.time,Motion.dq(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq4(rad/s)')

subplot(2,3,5)
plot(Motion.time,Motion.dq(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq5(rad/s)')

subplot(2,3,6)
plot(Motion.time,Motion.dq(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq6(rad/s)')

set(gcf,'NextPlot','add');
axes;
h = title('Joints Velocities');
set(gca,'Visible','off');
set(h,'Visible','on');

%% Plot the joints accelerations
figure(3)

subplot(2,3,1)
plot(Motion.time,Motion.ddq(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq1(rad/s)')

subplot(2,3,2)
plot(Motion.time,Motion.ddq(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq2(rad/s)')

subplot(2,3,3)
plot(Motion.time,Motion.ddq(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq3(cm/s)')

subplot(2,3,4)
plot(Motion.time,Motion.ddq(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq4(rad/s)')

subplot(2,3,5)
plot(Motion.time,Motion.ddq(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq5(rad/s)')

subplot(2,3,6)
plot(Motion.time,Motion.ddq(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq6(rad/s)')

set(gcf,'NextPlot','add');
axes;
h = title('Joints Accelerations');
set(gca,'Visible','off');
set(h,'Visible','on');
