clear
close all

%% Set PWM flag
% Set PWM flag: 1 to enable PWM voltage, 0 to disable and use ideal voltage source
ENABLE_PWM = 0;

load_system('six_phase_plant') % load Simulink model
load_system('MP_BearinglessMotor') % load Simulink model

%% Sampling time
if ENABLE_PWM == 0
    Tsim = 1e-5; % Simulation sampling time
    % Comment out PWM voltage inverter block
    set_param('MP_BearinglessMotor/Voltage Source Inverter/PWM Voltage Source Inverter', 'Commented', 'on');
    set_param('MP_BearinglessMotor/Voltage Source Inverter/Ideal Voltage Source Inverter', 'Commented', 'off');
elseif ENABLE_PWM == 1
    Tsim = 1e-8; % Simulation sampling time
    % Comment out ideal voltage source inverter block
    set_param('MP_BearinglessMotor/Voltage Source Inverter/PWM Voltage Source Inverter', 'Commented', 'off');
    set_param('MP_BearinglessMotor/Voltage Source Inverter/Ideal Voltage Source Inverter', 'Commented', 'on'); 
end

Tend = 10; % Simulation stop time

V_DC = 160; % DC bus (V)

torque_cmd = 0; % Torque reference (Nm)
torque_start = 0.002; % Torque start time (s)
torque_end = 0.025; % Torque end time (s)

fx_cmd = 0; % fx force reference (N)
fx_start = 0.002; % x-axis force start time (s)
fx_end = 0.025; % x-axis force end time (s)
fy_cmd = 0; % fy force reference (N)
fy_start = 0.001; % y-axis force start time (s)
fy_end = 0.025; % y-axis force end time (s)

id_cmd = 0; % id current reference (A)
id_start = 0; % d-axis current start time (s)
id_end = 0.025; % d-axis current end time (s)

speed_cmd = 0; % Speed command (r/min)

%% Signal injection
f_init = 1; % Initial frequency of chirp signal (Hz)
f_target = 1e3; % Traget frequency of chirp signal (Hz)
vd_inj = 1; % Amplitude of d-axis voltage chirp (V)

%% Plant parameters
p = 4; % Number of torque pole-pair
ps = 5; % Number of suspension pole-pair

m = 6;  % Number of phase
alpha_t = 2*pi/m*p;
alpha_s = 2*pi/m*ps;

R = 0.3; % Phase resistance (Ohm)
Rmat = R*eye(6, 6);

% Leakage inductance 1%
Lt = 300e-6; % Self-inductance of torque system without leakage
Ls = 450e-6; % Self-inductance of suspension system without leakage
Llkg = 0.01*Lt;

% calculate Generalized Clarke Transform
Cm = 2/m;
Cr = Cm*[cos(0*alpha_t) cos(1*alpha_t) cos(2*alpha_t) cos(3*alpha_t) cos(4*alpha_t) cos(5*alpha_t);
         sin(0*alpha_t) sin(1*alpha_t) sin(2*alpha_t) sin(3*alpha_t) sin(4*alpha_t) sin(5*alpha_t);
         cos(0*alpha_s) cos(1*alpha_s) cos(2*alpha_s) cos(3*alpha_s) cos(4*alpha_s) cos(5*alpha_s);
         sin(0*alpha_s) sin(1*alpha_s) sin(2*alpha_s) sin(3*alpha_s) sin(4*alpha_s) sin(5*alpha_s);
         1 1 1 1 1 1;
         1 -1 1 -1 1 -1;];

% Either calculate inductances from synchronous frame or stationary frame
% L_alpha_beta = [Lt+Llkg 0 0 0 0 0;
%                 0 Lt+Llkg 0 0 0 0;
%                 0 0 Ls+Llkg 0 0 0;
%                 0 0 0 Ls+Llkg 0 0;
%                 0 0 0 0 Llkg 0;
%                 0 0 0 0 0 Llkg;];
% 

% Lmat = round((Cr\L_alpha_beta)*Cr, 7);

% The Lmat matrix above is equivalent to the expression below:
Lmat = 2/m*[Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(5*alpha_t)+Ls*cos(5*alpha_s);
            Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s);
            Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s);
            Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s);
            Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls                               Lt*cos(alpha_t)+Ls*cos(alpha_s);
            Lt*cos(5*alpha_t)+Ls*cos(5*alpha_s) Lt*cos(4*alpha_t)+Ls*cos(4*alpha_s) Lt*cos(3*alpha_t)+Ls*cos(3*alpha_s) Lt*cos(2*alpha_t)+Ls*cos(2*alpha_s) Lt*cos(alpha_t)+Ls*cos(alpha_s)     Lt+Ls;] ...
        + Llkg*eye(6);

% Need to update Lt and Ls to include Llkg 
L_alpha_beta = Cr*Lmat/Cr;
Lt = L_alpha_beta(1,1);
Ls = L_alpha_beta(3,3);

%% Machine constants
Ke = 0;  % back-emf constant (Vpk/(mech rad/s))
Kt = 0.02; % Torque constant (Nm/Apk)
Kf = 1.8; % Force constant (N/Apk)

tau_hat = 2/m*Kt;

Fx_hat = 2/m*Kf;
if ps > p % ps = p + 1
    Fy_hat = - 2/m*Kf;
elseif ps < p % ps = p - 1
    Fy_hat = 2/m*Kf;
end

%% Power electronics
Fsw = 88000; % Inverter switching frequency (Hz)
Tsw = 1/Fsw; % Inverter switching period (sec)

%% Controller
fb = 500; % Bandwidth (Hz)
wb = 2*pi*fb; % Bandwidth (rad/s)

Kp_t = Lt*wb; % Torque current regulation P gain
Ki_t = R*wb; % Torque current regulation I gain

Kp_s = Ls*wb; % Suspension current regulation P gain
Ki_s = R*wb; % Suspension current regulation I gain

%% Run simulation
out = sim('six_phase_plant.slx');

%% Post processing
% Extract simulation data 
runObj = Simulink.sdi.Run.getLatest;

% List of variables to extract
obj2ext = {'time','vd_inj','id'};

% Get signal IDs and store signals into array
for idx = 2:length(obj2ext)
    sigID = getSignalIDsByName(runObj,obj2ext{idx});
    sig_obj.(obj2ext{idx}) = Simulink.sdi.getSignal(sigID);
    sig_val.(obj2ext{idx}) = sig_obj.(obj2ext{idx}).Values.Data;
end

time = sig_obj.(obj2ext{2}).Values.Time;

%% Select Data for FRF
num = squeeze(sig_val.id);
den = squeeze(sig_val.vd_inj);

%% Generate FRF Data
lines = 1e5;
win = 'hann'; % try 'hann' or 'rectwin'

[freq, mag, phase, coh] = generateFRF(num, den, Tsim, lines, win);

%% Estimate RL
% Find where frequency goes positive
idx_f_pos = find(freq >= 0, 1);

% Create data object of only positive freq response
data = frd(mag(idx_f_pos:end).*exp(1j*deg2rad(phase(idx_f_pos:end))), 2*pi*freq(idx_f_pos:end));

% Use the data object to estimate the transfer function
num_poles = 1;
sys = tfest(data, num_poles);

% Print sys to console to see how well it worked
sys

[Z,gain] = zero(sys);
P = pole(sys)

Rp_estimated = -P/gain;
Lp_estimated = 1/gain;

fprintf('Estimated resistance: %0.8f, actual: %0.8f, error: %f pct\n', ...
    Rp_estimated, R, 100 * (Rp_estimated - R)/R);
fprintf('Estimated inductance: %0.8f, actual: %0.8f, error: %f pct\n', ...
    Lp_estimated, Lt, 100 * (Lp_estimated - Lt)/Lt);

%% Plot figure
width = 2*5.43; 
height = 1.2*3*4.38 / 3;
set(0,'units','inches');
Inch_SS = get(0,'screensize');
lw = 1;  % line width

figure1 = figure;
% Plot voltage
subplot(2,1,1);
hold on;
xlabel('Time [s]','Interpreter','latex');
plot(time, squeeze(sig_val.vd_inj), 'Color', 'k', 'LineWidth', lw);
ylabel('Voltage (V)','Interpreter','latex');
legend('v_d','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

% Plot current
subplot(2,1,2);
hold on;
xlabel('Time [s]','Interpreter','latex');
plot(time, squeeze(sig_val.id), 'Color', 'k', 'LineWidth', lw);
ylabel('Current (A)','Interpreter','latex');
legend('i_d','Interpreter','latex','Location','east');
xlim([0 Tend]);
% ylim([-2.5 2.5]);

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

set(figure1,'Units','inches','Position',[(Inch_SS(3)-width)/2 (Inch_SS(4)-height)/2 width height]);
print(figure1, '-dsvg','-noui','plot_results');
print(figure1, '-dpng','-r300','plot_results');

%% Plot Estimation

% Show coherence plot?
% 1: yes
% 0: no
SHOW_COHERENCE = 1;

figure2 = figure;

if SHOW_COHERENCE == 1
    tiledlayout(3,1);
    ax1 = nexttile;
    ax2 = nexttile;
    ax3 = nexttile;
else
    tiledlayout(2,1);
    ax1 = nexttile;
    ax2 = nexttile;
end

markersize = 8;

plot(ax1, freq, mag, '.', 'markersize', markersize);
plot(ax2, freq, phase, '.', 'markersize', markersize);

hold (ax1, 'on'); 
hold (ax2, 'on'); 

% Set ideal transfer function here to overlay the curve fit
Gp = tf([0 1], [Lt R]);
[mag_ideal, phase_ideal] = bode(Gp, freq*2*pi);
plot(ax1, freq, squeeze(mag_ideal), 'r');
plot(ax2, freq, squeeze(phase_ideal), 'r');

if SHOW_COHERENCE == 1
    plot(ax3, freq, coh, '.', 'markersize', markersize);
end

ylabel(ax1, "Magnitude (-)");
ylabel(ax2, "Phase (deg)");
if SHOW_COHERENCE == 1
    ylabel(ax3, "Coherence");
end

if SHOW_COHERENCE == 1
    xlabel(ax3, "Frequency (Hz)");
else
    xlabel(ax2, "Frequency (Hz)");
end

f1 = f_init;
f2 = f_target;
xlim(ax1, [f1 f2]);
xlim(ax2, [f1 f2]);

grid(ax1, 'on');
grid(ax2, 'on');

set(ax1, 'xscale', 'log');
set(ax2, 'xscale', 'log');

% ylim(ax1, [0 6]);
% ylim(ax2, [-120 10]);

legend(ax1, {'FRF obtained from using tfestimate', 'Ideal FRF'}, 'Location', 'southwest');
legend(ax2, {'FRF obtained from using tfestimate', 'Ideal FRF'}, 'Location', 'southwest');

if SHOW_COHERENCE == 1
    xlim(ax3, [f1 f2]);
    ylim(ax3, [0 1.1]);
    grid(ax3, 'on');
    set(ax3, 'xscale', 'log');
end

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

print(figure2, '-dsvg','-noui','bode_plot');
print(figure2, '-dpng','-r300','bode_plot');

%% generateFRF function
function [freq, mag, phase, coh] = generateFRF(num, den, T, lines, win)

	fs = 1/T;
	overlap = lines/2;
	averages = floor(length(num)/(lines-overlap));
	windowType = window(win, lines);
	
	[FRF, freq] = tfestimate(den, num, windowType, overlap, lines, fs);
	[coh, freq] = mscohere(den, num, windowType, overlap, lines, fs);
	
	FRF = fftshift(FRF);
	coh = fftshift(coh);
	freq = freq - max(freq)/2;
	mag = abs(FRF);
	phase = angle(FRF) * 180/pi;

end