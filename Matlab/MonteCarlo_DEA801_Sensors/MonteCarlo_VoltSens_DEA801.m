%% Voltage Sensor Monte Carlo - DEA801
clear; clc; close all;

disp(" *** Statistical Simulation *** ")
disp(" **** Voltage Sensor DEA801 **** ")
fprintf("\n")

N = 1000000;   % Samples

R_top_nom = 88.7;   % [KOhm] cada resistor de cima
R_bot_nom = 3.32;   % [KOhm] resistor de baixo

% Tolerância
tol = 0.01;           % 1%
sigma = tol/3;        % 1% = 3σ

% SW gain
G_sw = 0.007430;
Vdc = 325;

%% Samples Generation
% 5 resistors (upside)
R_top_samples = R_top_nom .* (1 + sigma .* randn(N,5));
R_top_total = sum(R_top_samples, 2);

% Resistor (botton)
R_bot_samples = R_bot_nom .* (1 + sigma .* randn(N,1));

%% Gain Calc for each sample & Vdc voltage calc
G = R_bot_samples ./ (R_top_total + R_bot_samples);
Est_Bus = G * (Vdc/G_sw); % Vdc Bus Voltage Calculated in the FW for 325Vdc.

%% Stats
V_mean = mean(Est_Bus);
V_std  = std(Est_Bus);

erro_rel = (Vdc - Est_Bus) ./ Est_Bus;   % Relative error (%)

%% Results
disp(" *** Monte Carlo *** ")
fprintf("Mean Vbus: %.2f\n", V_mean);
fprintf("Std. Dev.: %.3f\n", V_std);
fprintf("Relative Mean Error vs SW: %.3f%%\n", mean(erro_rel)*100);
fprintf("Relative Max. Error vs SW: %.3f%%\n\n", max(abs(erro_rel))*100);

%% Plot
figure;
histogram(Est_Bus, 1000);
xlabel('Gain');
ylabel('Counts');
title('Estimated DC Bus Voltage Distribution @ 325Vdc - Monte Carlo');
grid on;


%% Limit Specification
tol = 0.02;  % 2% tolerance
LSL = Vdc * (1 - tol);
USL = Vdc * (1 + tol);

%% Cp e Cpk Calc
Cp  = (USL - LSL) / (6 * V_std);
Cpk_d = (USL - V_mean) / (3 * V_std);
Cpk_u = (V_mean - LSL) / (3 * V_std);
Cpk = min(Cpk_d,Cpk_u);

%% Results
disp(" *** Cp & Cpk Analysis *** ")
fprintf("Tolerance = %.1f%%\n", (0.02*100));
fprintf("LSL = %.2f\n", LSL);
fprintf("USL = %.2f\n", USL);
fprintf("Cp  = %.3f\n", Cp);
fprintf("Cpk_u = %.3f\n", Cpk_u);
fprintf("Cpk_d = %.3f\n", Cpk_d);
fprintf("Cpk = %.3f\n", Cpk);

disp(" *** End of Simulation *** ")