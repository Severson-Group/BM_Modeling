close all
clear all
clc

% DQ Motor Model parameters
Ld = 7e-3; % [H]
Lq = 5e-3; % [H]

Llkg = 0.2e-3;
L0 = (Ld + Lq - (2*Llkg)) / 3;
Lg = (Ld - Lq) / 3;

R = 0.3; % [ohms]
Ke = 0.1; % [V / rad/s]
lambda_pm = 0.1; % [wb]

% DQ Transform type
c = 2/3; % 2/3 means amplitude invariant

% SFPI controller tuning
wb_i = 2*pi * 200; % [rad/s]
Kp_d = wb_i * Ld;
Ki_d = wb_i * R;
Kp_q = wb_i * Lq;
Ki_q = wb_i * R;

% Operating Point (models 1-4)
OMEGA_E = 2*pi*60; % [rad/s]

% Gains
CROSS_COUPLING_DECOUPLING_GAIN = 1;
BACK_EMF_DECOUPLING_GAIN = 1;

% PWM VSI parameters
V_dc = 200; % [V]
f_sw = 50e3; % [Hz]

% Simulation settings
t_end = 0.3; % [sec]
solver_tol = 1e-4;
t_step_max = (1/f_sw) / 5;

