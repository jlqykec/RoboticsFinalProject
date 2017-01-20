function plotCartEvolution(Motion)
figure()
%Plot X position, velocity and acceleration
subplot(3,3,1)
plot(Motion.time(),Motion.x)
title('position of x');
grid on
subplot(3,3,2)
plot(Motion.time(1:end-1),Motion.dx)
title('velocity of x');
grid on
subplot(3,3,3)
plot(Motion.time(1:end-2),Motion.ddx)
title('acceleration of x');
grid on

%Plot Y position, velocity and acceleration
subplot(3,3,4)
plot(Motion.time(),Motion.y)
title('position of y');
ylabel('Position(cm)');
grid on
subplot(3,3,5)
plot(Motion.time(1:end-1),Motion.dy)
title('velocity of y');
ylabel('Velocity(cm/s)');
grid on
subplot(3,3,6)
plot(Motion.time(1:end-2),Motion.ddy)
title('acceleration of y');
ylabel('Acceleration(cm/s^2)');
grid on

%Plot Z position, velocity and acceleration
subplot(3,3,7)
plot(Motion.time(),Motion.z)
title('position of z');
grid on
subplot(3,3,8)
plot(Motion.time(1:end-1),Motion.dz)
title('velocity of z');
grid on
subplot(3,3,9)
plot(Motion.time(1:end-2),Motion.ddz)
title('acceleration of z');
grid on
end