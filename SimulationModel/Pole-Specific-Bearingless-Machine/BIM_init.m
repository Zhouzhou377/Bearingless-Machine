clc;
clear all


%% Machine Parameters


R_s = 0.25*2;                        % Stator Resistance [Ohm]
L_sigma_s = 0.74e-3*2;                % Stator leakage Inductance [H]
R_r = 0.47;                          % Rotor Resistance [Ohm]
L_sigma_r = 0.74e-3;                % Rotor leakage Inductance [H]
L_m = 5.02e-3;                    % Magnetizing Inductance [H]
L_s = L_m + L_sigma_s/2;              % Stator Inductance [H]
L_r = L_m + L_sigma_r;              % Rotor Inductance [H]

J = 0.005;                           % Rotor Inertia [kg*m^2]
b = 5.879e-4*0.05;                       % Viscous Damping [N*m/(rad/s)]
Tf = 0;                             % Static Friction [N*m]
PolePair = 1;                       % Number of Pole Pairs

tau_r = 0.012255319148936;
LS = 0.2850e-3;

Lmp = 2/3*L_m;
Lms = 2/3*LS;

%% Calculated Machine Parameters

L_sigma_S = L_sigma_s;
L_sigma_R = 0;
R_R = (L_m/L_r)^2*R_r;
L_M = L_m*L_m/L_r;

%%
% kf = 0.0130322*id*id+0.797446*id+0.0190783;
kf = 40/3;
k_delta = 70000;

l_airgap = 2.65e-3;
m_rotor = 1.5;

%%
% Udc = 380*sqrt(3); 
Udc = 100;
fsw = 10e3;
T = 1/fsw;

%%
Ts = 1/fsw;

r_t_s = R_s/2;


Ap_t_s = 0.995669127615556;
Bp_t_s = 0.118088621701824;

r_s1_s = 2*R_s;

Ap_s1_s =0.975904794542392;
Bp_s1_s = 0.269597308951354;


Ad_t_s = 0.881911378298176;
Bd_t_s = 0.118088621701824;
Ad_s1_s = 0.730402691048646;
Bd_s1_s = 0.269597308951354;


C_dc = 1000e-6;

Ctr_v_Kp = 1;
Ctr_v_Ki = Ctr_v_Kp*0.1;

