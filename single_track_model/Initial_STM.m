%--------------------------------------------------------------------------
%
% Master Thesis - Trajectory Planning for the Teleoperated Vehicle (Safety Corridor) 
% TU Berlin
%
% Author: Jingsheng Lyu
% Email: jingshenglyu@gmail.com
% web: https://jingshenglyu.github.io/
%--------------------------------------------------------------------------

clear all;
close all;
clc;

disp('###  Single Track Model  ###')
disp(' ')
disp(' ~ Starte ~ ');
disp(' ');

%% --- Initialization and calculations --- 
disp(' ... Initial Parameter ... ')
disp(' ')
pause(2)

% given parameters
m_v = 749.8;                    % [kg]        Achslast vorn
m_h = 750.0;                    % [kg]        Achslast hinten
m = m_v + m_h;                  % [kg]        Gesamtmasse des Fahrzeugs
T_z = 2080.4;                   % [kg*m²]     Traegheitsmoment um z-Achse
L = 2.71;                       % [m]         Radstand
EG = 0.25;                      % [Grad*s²/m] Eigenlenkgradient    
SG = 0.30;                      % [Grad*s²/m] Schwimmwinkelgradient
mu = 0.9;                       % [-]         Reibungskoeffizient
C  = 1.3;                       % [-]         Formfaktor fuer Magic Formula
E  = 0.85;                      % [-]         Kruemmungsfaktor fuer Magic Formula

% Parametrierung der Lenkung
Lenkwinkel = [-405 -180 -90 -45 0 45 90 180 410];
Radlenkwinkel = [-34.5 -10.5 -5.5 -3 0 3 5.5 10.5 35.5];

% Initialdefinition
i_L = 16;                       % [-]         Lenkuebersetzung

%% calculations!
% Legende: SP Schwerpunkt | VA Vorderachse | HA Hinterachse | CS
% Corneringstiffness

l_h = m_v * L / m;              % [m]       Berechnung SP-Lage HA
l_v = m_h * L / m;              % [m]       Berechnung SP-Lage VA

c_h = m_h / SG;                 % [N/grad]   Berechnung CS HA
c_v = l_h / (( L * EG / m ) + l_v / c_h );   % [N/grad] Berechnung CS VA

c_v = c_v * 180 / pi;           % [N/grad] => [N/rad]
c_h = c_h * 180 / pi;           % [N/grad] => [N/rad]

D_v = mu * m_v * 9.81;          % [1]       Berechnung Parameter D VA
D_h = mu * m_h * 9.81;          % [1]       Berechnung Parameter D HA

B_v = c_v / (C * D_v);          % [1]       Berechnung Parameter B VA
B_h = c_h / (C * D_h);          % [1]       Berechnung Parameter B HA

% tsim [s] | delta [°] | v [m/s] | set_delta [bool] | set_v [bool] | mode_tire [bool]
setting = [120 90 2.77 0 1 0; 120 90 2.77 0 1 1; 90 0 25 1 0 0; 90 0 25 1 0 1];
row = size(setting,1);
data = cell([row 11]);

disp(' ~ Initialization completed ~ ')
disp(' ')
pause(2);

% --- Simulation STM: Loop over driving maneuver
disp(' ~ Start simulation process single track model ~ ')
disp(' ')
pause(2)

for i = 1:4
    if i < 3
        disp(strcat(' > Start Simulation',{' '},num2str(i),'/4: Driving manoeuvres 1'))
    elseif i > 2
        disp(strcat(' > Start Simulation',{' '},num2str(i),'/4: Driving manoeuvres 2'))
    end
    
    % Simulation Setting
    t_sim = setting(i,1);
    set_delta = setting(i,4); % [bool] 
    set_v = setting(i,5); 
    Mode_Tire = setting(i,6);
    
    % Simulation Initalwert
    delta = setting(i,2);
    v = setting(i,3);
    
    a_y_init = 0.2695;
    R_init = 28.62;
    psi_p_init = 0;
    beta_init = 0;    
    
    sim single_track_model.slx
    
    R_out(R_out>=1000)=0;
    
    disp(' ... Save results ... ')
    
    data{i,1} = delta_out;
    data{i,2} = v_out;
    data{i,3} = a_y_out;
    data{i,4} = psi_p_out;
    data{i,5} = beta_out;
    data{i,6} = alpha_v_out;
    data{i,7} = alpha_h_out;
    data{i,8} = R_out;
    data{i,9} = x_out;
    data{i,10} = y_out;
    data{i,11} = t_out;
    
    disp(' Results saved ')
    disp(' ')
    pause(2)    
end