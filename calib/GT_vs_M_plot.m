function GT_vs_M_plot(GT_t, GT, Measured_t, Measured, name)

figure
title(strcat(name, 'Ground Truth'))
subplot(3, 1, 1)
plot(GT_t, GT(1, :));
subplot(3, 1, 2)
plot(GT_t, GT(2, :));
subplot(3, 1, 3)
plot(GT_t, GT(3, :));



figure
title(strcat(name, 'Measured'))
subplot(3, 1, 1)
plot(Measured_t, Measured(1, :));
subplot(3, 1, 2)
plot(Measured_t, Measured(2, :));
subplot(3, 1, 3)
plot(Measured_t, Measured(3, :));

end

