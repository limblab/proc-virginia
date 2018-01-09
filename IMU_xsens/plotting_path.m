function[]=plotting_path(X_elb)

x_elb = X_elb(1:5:end,1);
y_elb = X_elb(1:5:end,2);
z_elb = X_elb(1:5:end,3);

figure
set(gcf, 'units', 'normalized', 'position', [0.1 0.1 0.8 0.8])

for ii = 1:length(x_elb)
    subplot(221)
    plot(x_elb(ii),y_elb(ii),'b*')
    xlabel('x'); ylabel('y');
    grid on; hold on
    axis([min(x_elb) max(x_elb) min(y_elb) max(y_elb)]);
    
    subplot(222)
    plot(y_elb(ii),z_elb(ii),'b*')
    xlabel('y'); ylabel('z');
    grid on; hold on
    axis([min(y_elb) max(y_elb) min(z_elb) max(z_elb)]);
    
    subplot(223)
    plot(x_elb(ii),z_elb(ii),'b*')
    xlabel('x'); ylabel('z');
    grid on; hold on
    axis([min(x_elb) max(x_elb) min(z_elb) max(z_elb)]);

    subplot(224)
    plot3(x_elb(ii),y_elb(ii),z_elb(ii),'b*')
    xlabel('x'); ylabel('y'); zlabel('z');
    grid on; hold on
    axis([min(x_elb) max(x_elb) min(y_elb) max(y_elb) min(z_elb) max(z_elb)]);
    
drawnow;
end
end