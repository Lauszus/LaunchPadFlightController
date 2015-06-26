clear all
close all
clc

hold on
if 0
    data = load('acro/acro_manual.txt');
    ylabel('Rate [deg/s]', 'fontsize', 15);
elseif 0
    data = load('self_level/self_level_30.txt');
    ylabel('Angle [deg]', 'fontsize', 15);
elseif 1
    data = load('heading/heading_45deg_10s.txt');
    for i=1:length(data(:,4)) % Normalize data
        if (data(i,4) < -180)
            data(i,4) = data(i,4) + 360;
        elseif (data(i,4) > 180)
            data(i,4) = data(i,4) - 360;
        end
    end
    ylabel('Angle [deg]', 'fontsize', 15);
else
    convert = 1;
    if convert == 1
        data = load('altitude/altitude_step.txt');
        ylabel('Height [mm]', 'fontsize', 15);

        % Used to convert the values back to height
        MIN_MOTOR_OUT = -100;
        MAX_MOTOR_OUT = 100;
        SONAR_MIN_DIST = 50;
        SONAR_MAX_DIST = 1500;
        data(:,3) = mapf(data(:,3), MIN_MOTOR_OUT, MAX_MOTOR_OUT,...
                                           SONAR_MIN_DIST, SONAR_MAX_DIST);
        data(:,4) = mapf(data(:,4), MIN_MOTOR_OUT, MAX_MOTOR_OUT,...
                                           SONAR_MIN_DIST, SONAR_MAX_DIST);
    else
        data = load('altitude/altitude_step.txt');
    end
end

time = data(:,2) / 1e6; % Convert to seconds
setPoint = data(:,3);
input = data(:,4);

figure(1)
plot(time, setPoint, 'r');
figure1 = plot(time, input, 'b+');
legend('Set point', 'Input')
xlim([min(time) max(time)])
xlabel('Time [s]', 'fontsize', 15);
hold off

saveas(figure1, strcat(pwd, '/img/figure1'),'epsc') % Save figure