function [ ] = PrintPlot( plt)

% some predefined plots
% you can add your own

fig = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

fig = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');



fig = figure('Name', 'Activation Functions Left Arm');
hplot = plot(plt.t, plt.A(1:4,:));
set(hplot, 'LineWidth', 2);
legend('Joint Limits', 'Minimal Altitude', 'Tool', 'Cooperation');

fig = figure('Name', 'Activation Functions Right Arm');
hplot = plot(plt.t, plt.A2(1:4,:));
set(hplot, 'LineWidth', 2);
legend('Joint Limits', 'Minimal Altitude', 'Tool', 'Cooperation');





fig = figure('Name', 'the desired object velocity and the non-cooperative Cartesian velocities Left Arm');
subplot(2,1,1);
title('the desired object velocity');
hplot = plot(plt.t, plt.xdot);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('the non-cooperative Cartesian velocities');
hplot = plot(plt.t, plt.qnoncoop);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');

fig = figure('Name', 'the desired object velocity and the non-cooperative Cartesian velocities Right Arm');
subplot(2,1,1);
title('the desired object velocity');
hplot = plot(plt.t, plt.xdot2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('the non-cooperative Cartesian velocities');
hplot = plot(plt.t, plt.qnoncoop2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');




fig = figure('Name', 'the desired object velocity and the cooperative Cartesian velocities Left Arm');
subplot(2,1,1);
title('the desired object velocity');
hplot = plot(plt.t, plt.xdot);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('the cooperative Cartesian velocities');
hplot = plot(plt.t, plt.xdotcoop2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');

fig = figure('Name', 'the desired object velocity and the cooperative Cartesian velocities Right Arm');
subplot(2,1,1);
title('the desired object velocity');
hplot = plot(plt.t, plt.xdot2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('the cooperative Cartesian velocities');
hplot = plot(plt.t, plt.xdotcoop2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');


end

