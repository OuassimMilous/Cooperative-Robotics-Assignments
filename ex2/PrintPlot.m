function [ ] = PrintPlot( plt, pandaArm )

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



fig = figure('Name', 'Activation Function Left Arm');
hplot = plot(plt.t, plt.A(1:4,:));
set(hplot, 'LineWidth', 2);
legend('Joint Limits', 'Minimal Altitude', 'Tool', 'Constraint');

fig = figure('Name', 'Activation Function Right Arm');
hplot = plot(plt.t, plt.A2(1:4,:));
set(hplot, 'LineWidth', 2);
legend('Joint Limits', 'Minimal Altitude', 'Tool', 'Constraint');





fig = figure('Name', 'End Effector AND desired position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.xdot);
title('End Effector');
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('desired');
hplot = plot(plt.t, plt.qe_dot);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');

fig = figure('Name', 'End Effector AND desired position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.xdot2);
title('End Effector');
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');
subplot(2,1,2);
title('desired');
hplot = plot(plt.t, plt.qe_dot2);
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw','x','y','z');

end

