% build_simulink_model.m
% Constructs the 230 kV, 120 km Two-Bus Power System Transmission Model
% in Simulink (Simscape Electrical / Specialized Power Systems)
% used as the data source for fault classification verification in EE 315 Assignment 12.
%
% Line parameters are based on EE 315 Lecture 35 (Slides 17-18):
%   z1 = 0.02 + j0.28678 Ohm/km (r1 = 0.02 Ohm/km, L1 = 0.91285 mH/km)
%   z0 = 0.10645 + j0.836989 Ohm/km (r0 = 0.10645 Ohm/km, L0 = 2.6642 mH/km)
% Line capacitance assumption (for physically valid propagation speed < c):
%   c1 = 14.0 nF/km, c0 = 7.75 nF/km (v1 ~ 280,000 km/s, v0 ~ 220,000 km/s)

modelName = 'power_network_model';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
open_system(modelName);

% Load Simscape Specialized Power Systems library blocks
load_system('spspowerguiLib');
load_system('spsThreePhaseSourceLib');
load_system('spsThreePhasePISectionLineLib');
load_system('spsThreePhaseFaultLib');
load_system('spsThreePhaseVIMeasurementLib');

% 1. Powergui Block (Discrete solver, Ts = 50 us)
add_block('spspowerguiLib/powergui', [modelName, '/powergui'], ...
    'Position', [50, 40, 130, 80], ...
    'SimulationMode', 'Discrete', ...
    'SampleTime', '5e-5');

% 2. Three-Phase AC Source at Bus M (230 kV L-L RMS, 50 Hz, 0 deg)
% Source impedance assumed as Zs_M = 0.5 + j5.0 Ohm (R = 0.5 Ohm, L = 0.0159 H)
h_sm = add_block('spsThreePhaseSourceLib/Three-Phase Source', [modelName, '/Source_M'], ...
    'Position', [50, 180, 130, 240], ...
    'Voltage', '230e3', 'Frequency', '50', 'PhaseAngle', '0', ...
    'SpecifyImpedance', 'on', 'Resistance', '0.5', 'Inductance', '0.0159');

% 3. Three-Phase V-I Measurement at Bus M (Relay Location acquiring line currents)
h_vi = add_block('spsThreePhaseVIMeasurementLib/Three-Phase V-I Measurement', [modelName, '/Relay_M_VI'], ...
    'Position', [220, 180, 280, 240], ...
    'VoltageMeasurement', 'phase-to-ground', ...
    'CurrentMeasurement', 'yes');

% 4. Transmission Line Section 1 (Bus M to Fault Location F, default length = 30 km)
h_l1 = add_block('spsThreePhasePISectionLineLib/Three-Phase PI Section Line', [modelName, '/Line_Section_1'], ...
    'Position', [360, 190, 450, 230], ...
    'Frequency', '50', ...
    'Resistances', '[0.02, 0.10645]', ...
    'Inductances', '[0.91285e-3, 2.6642e-3]', ...
    'Capacitances', '[14.0e-9, 7.75e-9]', ...
    'Length', '30');

% 5. Three-Phase Fault Block at F (configurable fault types and resistance)
h_flt = add_block('spsThreePhaseFaultLib/Three-Phase Fault', [modelName, '/Fault_F'], ...
    'Position', [520, 290, 570, 350], ...
    'FaultA', 'off', 'FaultB', 'off', 'FaultC', 'off', 'GroundFault', 'off', ...
    'SwitchTimes', '[0.04 0.10]', ...
    'FaultResistance', '1.0', ...
    'GroundResistance', '0.01');

% 6. Transmission Line Section 2 (Fault Location F to Bus N, default length = 90 km)
h_l2 = add_block('spsThreePhasePISectionLineLib/Three-Phase PI Section Line', [modelName, '/Line_Section_2'], ...
    'Position', [620, 190, 710, 230], ...
    'Frequency', '50', ...
    'Resistances', '[0.02, 0.10645]', ...
    'Inductances', '[0.91285e-3, 2.6642e-3]', ...
    'Capacitances', '[14.0e-9, 7.75e-9]', ...
    'Length', '90');

% 7. Three-Phase AC Source at Bus N (230 kV L-L RMS, 50 Hz, -10 deg load angle)
% Source impedance assumed as Zs_N = 0.5 + j5.0 Ohm (R = 0.5 Ohm, L = 0.0159 H)
h_sn = add_block('spsThreePhaseSourceLib/Three-Phase Source', [modelName, '/Source_N'], ...
    'Position', [780, 180, 860, 240], ...
    'Voltage', '230e3', 'Frequency', '50', 'PhaseAngle', '-10', ...
    'SpecifyImpedance', 'on', 'Resistance', '0.5', 'Inductance', '0.0159');

% 8. To Workspace Block for Three-Phase Currents (sampled at fs = 1000 Hz, Ts = 1 ms)
h_wi = add_block('simulink/Sinks/To Workspace', [modelName, '/ToWorkspace_I'], ...
    'Position', [350, 100, 420, 130], ...
    'VariableName', 'I_abc', 'SaveFormat', 'Array', 'SampleTime', '1e-3');

% 9. Scope Block for Three-Phase Currents at Relay Bus M
h_sc = add_block('simulink/Sinks/Scope', [modelName, '/Scope_Relay'], ...
    'Position', [350, 40, 390, 80]);

% Connect physical electrical lines using block port handles
p_sm  = get_param(h_sm,  'PortHandles');
p_vi  = get_param(h_vi,  'PortHandles');
p_l1  = get_param(h_l1,  'PortHandles');
p_flt = get_param(h_flt, 'PortHandles');
p_l2  = get_param(h_l2,  'PortHandles');
p_sn  = get_param(h_sn,  'PortHandles');
p_wi  = get_param(h_wi,  'PortHandles');
p_sc  = get_param(h_sc,  'PortHandles');

% Source M to V-I measurement
for p = 1:min(length(p_sm.RConn), length(p_vi.LConn))
    add_line(modelName, p_sm.RConn(p), p_vi.LConn(p));
end

% V-I measurement to Line Section 1
for p = 1:min(length(p_vi.RConn), length(p_l1.LConn))
    add_line(modelName, p_vi.RConn(p), p_l1.LConn(p));
end

% Line Section 1 to Line Section 2
for p = 1:min(length(p_l1.RConn), length(p_l2.LConn))
    add_line(modelName, p_l1.RConn(p), p_l2.LConn(p));
end

% Fault block to Line 1 tap point F
for p = 1:min(length(p_flt.LConn), length(p_l1.RConn))
    add_line(modelName, p_flt.LConn(p), p_l1.RConn(p));
end

% Line Section 2 to Source N
for p = 1:min(length(p_l2.RConn), length(p_sn.RConn))
    add_line(modelName, p_l2.RConn(p), p_sn.RConn(p));
end

% Connect Current measurement signal (Outport 2) to ToWorkspace_I and Scope_Relay
if length(p_vi.Outport) >= 2
    add_line(modelName, p_vi.Outport(2), p_wi.Inport(1));
    add_line(modelName, p_vi.Outport(2), p_sc.Inport(1));
end

% Set simulation parameters (0.12 s total simulation time = 6 cycles of 50 Hz)
set_param(modelName, 'StopTime', '0.12', 'Solver', 'FixedStepDiscrete', 'FixedStep', '5e-5');

% Save and close model
slx_path = fullfile(pwd, [modelName, '.slx']);
save_system(modelName, slx_path);
close_system(modelName);

fprintf('Successfully constructed and saved Simulink model: %s\n', slx_path);
