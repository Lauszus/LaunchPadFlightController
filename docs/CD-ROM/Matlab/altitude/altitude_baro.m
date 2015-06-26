clear all
close all
clc

if 1
    data = load('altitude_indoor.txt');
else
    data = load('altitude_baro_10m.txt');
end

time = data(:,2) / 1e6; % Convert to seconds
time = time - time(1); % Subtract first value, so it starts at 0s

input_lpf = data(:,3);
input = data(:,4);

figure(1)
hold on
plot(time, input, 'g+');
figure1 = plot(time, input_lpf, 'b+');
legend('Input', 'Input LPF')
xlim([min(time) max(time)])
xlabel('Time [s]', 'fontsize', 15);
ylabel('Altitude cm', 'fontsize', 15);
hold off

saveas(figure1, strcat(pwd, '/img/figure1'),'epsc') % Save figure