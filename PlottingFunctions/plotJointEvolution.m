function plotJointEvolution(Motion)
%% Plot the joints positions
figure()

%q1
subplot(2,3,1)
plot(Motion.time,Motion.q(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q1(rad)')

%q2
subplot(2,3,2)
plot(Motion.time,Motion.q(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q2(rad)')

%q3
subplot(2,3,3)
plot(Motion.time,Motion.q(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q3(cm)')

%q4
subplot(2,3,4)
plot(Motion.time,Motion.q(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q4(rad)')

%q5
subplot(2,3,5)
plot(Motion.time,Motion.q(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q5(rad)')

%q6
subplot(2,3,6)
plot(Motion.time,Motion.q(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q6(rad)')

%Put the title on the top of the window
set(gcf,'NextPlot','add');
axes;
h = title('Joints Positions');
set(gca,'Visible','off');
set(h,'Visible','on');

%% Plot the joints velocities
figure()

%dq1
subplot(2,3,1)
plot(Motion.time,Motion.dq(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq1(rad/s)')

%dq2
subplot(2,3,2)
plot(Motion.time,Motion.dq(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq2(rad/s)')

%dq3
subplot(2,3,3)
plot(Motion.time,Motion.dq(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq3(cm/s)')

%dq4
subplot(2,3,4)
plot(Motion.time,Motion.dq(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq4(rad/s)')

%dq5
subplot(2,3,5)
plot(Motion.time,Motion.dq(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq5(rad/s)')

%dq6
subplot(2,3,6)
plot(Motion.time,Motion.dq(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq6(rad/s)')

%Put the title on the top of the window
set(gcf,'NextPlot','add');
axes;
h = title('Joints Velocities');
set(gca,'Visible','off');
set(h,'Visible','on');

%% Plot the joints accelerations
figure(3)

%ddq1
subplot(2,3,1)
plot(Motion.time,Motion.ddq(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq1(rad/s)')

%ddq2
subplot(2,3,2)
plot(Motion.time,Motion.ddq(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq2(rad/s)')

%ddq3
subplot(2,3,3)
plot(Motion.time,Motion.ddq(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq3(cm/s)')

%ddq4
subplot(2,3,4)
plot(Motion.time,Motion.ddq(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq4(rad/s)')

%ddq5
subplot(2,3,5)
plot(Motion.time,Motion.ddq(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq5(rad/s)')

%ddq6
subplot(2,3,6)
plot(Motion.time,Motion.ddq(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('ddq6(rad/s)')

%Put the title on the top of the window
set(gcf,'NextPlot','add');
axes;
h = title('Joints Accelerations');
set(gca,'Visible','off');
set(h,'Visible','on');
