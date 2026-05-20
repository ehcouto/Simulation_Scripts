%% Current Sensor Monte Carlo - DEA801
clear; clc; close all;

disp(" *** Statistical Simulation *** ")
disp(" **** Current Sensor DEA801 **** ")
fprintf("\n")

N = 10000;   % Samples

R113 = 22;   % [KOhm]
R114 = 22;   % [KOhm]
R115 = 30;   % [KOhm]
R116 = 30;   % [KOhm]
R117 = 1;    % [KOhm]
R118 = 1;    % [KOhm]
Rshunt = 0.1;   % [Ohm]

% Tolerância
tol = 0.01;           % 1%
sigma = tol/3;        % 1% = 3σ

% SW gain
G_sw = -R115/R113;
Ishunt = 500; %[mAmps]

%% Samples Generation
% 5 resistors (upside)
R115_mc = R115 .* (1 + sigma .* randn(N,1));

% Resistor (botton)
R113_mc = R113 .* (1 + sigma .* randn(N,1));

%% Gain Calc for each sample & Vdc voltage calc
G = -R115_mc./R113_mc;
Est_Curr = G * (Ishunt/G_sw); % Current Calculated for 0.5A current. 

%% Stats
V_mean = mean(Est_Curr);
V_std  = std(Est_Curr);

erro_rel = (Ishunt - Est_Curr) ./ Est_Curr;   % Relative error (%)

%% Results
disp(" *** Monte Carlo *** ")
fprintf("Mean Vbus: %.2f\n", V_mean);
fprintf("Std. Dev.: %.3f\n", V_std);
fprintf("Relative Mean Error vs SW: %.3f%%\n", mean(erro_rel)*100);
fprintf("Relative Max. Error vs SW: %.3f%%\n\n", max(abs(erro_rel))*100);

%% Plot
figure;
histogram(Est_Curr, 100);
xlabel('Phase Current [mA]');
ylabel('Counts');
title('Estimated Phase Current Distribution @ 500mA - Monte Carlo');
grid on;


%% Limit Specification
tol = 0.02;  % 2% tolerance
LSL = Ishunt * (1 - tol);
USL = Ishunt * (1 + tol);

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