% run_project_verification.m
% Verification Script for EE 315 Assignment 12: Fault Classification
%
% Runs every test case directly through the Simulink model ('power_network_model.slx')
% and evaluates fault_classifier.m on the acquired three-phase currents.
%
% Evaluated fault classes (11 classes):
%   'NO_FAULT', 'AG', 'BG', 'CG', 'AB', 'BC', 'CA', 'ABG', 'BCG', 'CAG', 'ABC'
%
% Tested variations across Simulink model:
%   - Fault locations:  x = 30 km (25% line), x = 80 km (66.7% line)
%   - Fault resistance: Rf = 0.5 Ohm (bolted), Rf = 10.0 Ohm (moderate resistance)

clc; clear; close all;
fprintf('========================================================================\n');
fprintf('  EE 315: Assignment 12 - Power System Fault Classifier Verification     \n');
fprintf('  Source of Data: Simulink Model (power_network_model.slx)              \n');
fprintf('========================================================================\n\n');

modelName = 'power_network_model';
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% Defined fault classes
classes = {'NO_FAULT', 'AG', 'BG', 'CG', 'AB', 'BC', 'CA', 'ABG', 'BCG', 'CAG', 'ABC'};
n_classes = length(classes);

% Fault configurations for Three-Phase Fault block
% [FaultA, FaultB, FaultC, GroundFault]
fault_switch_map = containers.Map();
fault_switch_map('NO_FAULT') = {'off', 'off', 'off', 'off'};
fault_switch_map('AG')       = {'on',  'off', 'off', 'on'};
fault_switch_map('BG')       = {'off', 'on',  'off', 'on'};
fault_switch_map('CG')       = {'off', 'off', 'on',  'on'};
fault_switch_map('AB')       = {'on',  'on',  'off', 'off'};
fault_switch_map('BC')       = {'off', 'on',  'on',  'off'};
fault_switch_map('CA')       = {'on',  'off', 'on',  'off'};
fault_switch_map('ABG')      = {'on',  'on',  'off', 'on'};
fault_switch_map('BCG')      = {'off', 'on',  'on',  'on'};
fault_switch_map('CAG')      = {'on',  'off', 'on',  'on'};
fault_switch_map('ABC')      = {'on',  'on',  'on',  'off'};

% Tested parameter ranges
locations = [30, 80];        % km from Bus M (Total length = 120 km)
resistances = [0.5, 10.0];   % Ohms

total_cases = 0;
correct_cases = 0;
misclassified_cases = 0;
confusion_matrix = zeros(n_classes, n_classes);

% Store sample waveforms for plotting
sample_waveforms = struct();

fprintf('Running Simulink test cases across all fault classes...\n');
fprintf('%-4s | %-9s | %-8s | %-7s | %-10s | %-10s | %-8s\n', ...
    'No.', 'True Type', 'Loc (km)', 'Rf (Ohm)', 'Sim Time', 'Pred Type', 'Result');
fprintf('------------------------------------------------------------------------\n');

% 1. Simulate NO_FAULT cases under distinct load flow conditions
load_angles = {'-10', '+5'}; % Case 1: Forward load (-10 deg), Case 2: Reverse load (+5 deg)
for nf_case = 1:2
    total_cases = total_cases + 1;
    true_label = 'NO_FAULT';
    
    sw = fault_switch_map(true_label);
    set_param([modelName, '/Fault_F'], ...
        'FaultA', sw{1}, 'FaultB', sw{2}, 'FaultC', sw{3}, 'GroundFault', sw{4});
    set_param([modelName, '/Source_N'], 'PhaseAngle', load_angles{nf_case});
    
    tic;
    simOut = sim(modelName);
    t_sim = toc;
    
    I_abc = simOut.I_abc;
    [pred_label, info] = fault_classifier(I_abc, 1000);
    
    true_idx = find(strcmp(classes, true_label));
    pred_idx = find(strcmp(classes, pred_label));
    
    if strcmp(true_label, pred_label)
        correct_cases = correct_cases + 1;
        result_str = 'MATCH';
    else
        misclassified_cases = misclassified_cases + 1;
        result_str = 'MISMATCH';
    end
    
    if ~isempty(pred_idx)
        confusion_matrix(true_idx, pred_idx) = confusion_matrix(true_idx, pred_idx) + 1;
    end
    
    fprintf('%-4d | %-9s | %-8s | %-7s | %-9.3fs | %-10s | %-8s\n', ...
        total_cases, true_label, 'N/A', 'N/A', t_sim, pred_label, result_str);
end

% Reset Source N to nominal load angle (-10 deg)
set_param([modelName, '/Source_N'], 'PhaseAngle', '-10');

% 2. Simulate 10 Fault Types across locations and resistances
fault_types_to_test = {'AG', 'BG', 'CG', 'AB', 'BC', 'CA', 'ABG', 'BCG', 'CAG', 'ABC'};

for f = 1:length(fault_types_to_test)
    true_label = fault_types_to_test{f};
    sw = fault_switch_map(true_label);
    
    for loc = locations
        set_param([modelName, '/Line_Section_1'], 'Length', num2str(loc));
        set_param([modelName, '/Line_Section_2'], 'Length', num2str(120 - loc));
        
        for rf = resistances
            total_cases = total_cases + 1;
            
            set_param([modelName, '/Fault_F'], ...
                'FaultA', sw{1}, 'FaultB', sw{2}, 'FaultC', sw{3}, 'GroundFault', sw{4}, ...
                'FaultResistance', num2str(rf));
            
            tic;
            simOut = sim(modelName);
            t_sim = toc;
            
            I_abc = simOut.I_abc;
            [pred_label, info] = fault_classifier(I_abc, 1000);
            
            % Save sample waveforms for report figure
            if strcmp(true_label, 'AG') && loc == 30 && rf == 0.5
                sample_waveforms.AG = I_abc;
            elseif strcmp(true_label, 'BCG') && loc == 30 && rf == 0.5
                sample_waveforms.BCG = I_abc;
            end
            
            true_idx = find(strcmp(classes, true_label));
            pred_idx = find(strcmp(classes, pred_label));
            
            if strcmp(true_label, pred_label)
                correct_cases = correct_cases + 1;
                result_str = 'MATCH';
            else
                misclassified_cases = misclassified_cases + 1;
                result_str = 'MISMATCH';
            end
            
            if ~isempty(pred_idx)
                confusion_matrix(true_idx, pred_idx) = confusion_matrix(true_idx, pred_idx) + 1;
            end
            
            fprintf('%-4d | %-9s | %-8d | %-7.1f | %-9.3fs | %-10s | %-8s\n', ...
                total_cases, true_label, loc, rf, t_sim, pred_label, result_str);
        end
    end
end

% -------------------------------------------------------------------------
% Print Summary and Confusion Matrix
% -------------------------------------------------------------------------
accuracy = (correct_cases / total_cases) * 100;

fprintf('\n========================================================================\n');
fprintf('  SIMULINK VERIFICATION RESULTS SUMMARY                                  \n');
fprintf('========================================================================\n');
fprintf('Total Cases Simulated in Simulink : %d\n', total_cases);
fprintf('Correctly Classified              : %d\n', correct_cases);
fprintf('Misclassifications                : %d\n', misclassified_cases);
fprintf('Overall Classification Accuracy   : %.2f%%\n', accuracy);
fprintf('------------------------------------------------------------------------\n\n');

fprintf('Confusion Matrix:\n');
fprintf('Rows: True Class | Columns: Predicted Class\n\n');

fprintf('%-10s', 'True \ Pred');
for j = 1:n_classes
    fprintf('%7s', classes{j});
end
fprintf('\n');
fprintf('%s\n', repmat('-', 1, 10 + n_classes * 7));

for i = 1:n_classes
    fprintf('%-10s', classes{i});
    for j = 1:n_classes
        fprintf('%7d', confusion_matrix(i, j));
    end
    fprintf('\n');
end

% -------------------------------------------------------------------------
% Generate and Save Publication-Quality Figures for Report
% -------------------------------------------------------------------------
fprintf('\nGenerating verification figures for report...\n');

% Fig 1: Schematic of Two-Bus Transmission System
f1 = figure('Visible', 'off', 'Position', [100, 100, 850, 380], 'Color', 'w');
axes('Position', [0.05, 0.1, 0.9, 0.8]);
hold on; axis off;
xlim([0, 10]); ylim([0, 5]);

rectangle('Position', [0.8, 2.1, 0.8, 0.8], 'Curvature', [1, 1], 'LineWidth', 2, 'EdgeColor', [0 0.447 0.741]);
text(1.2, 2.5, 'S_M', 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
line([1.6, 2.5], [2.5, 2.5], 'LineWidth', 2, 'Color', 'k');
line([2.5, 2.5], [1.2, 3.8], 'LineWidth', 4, 'Color', 'k');
text(2.5, 4.1, 'Bus M', 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

rectangle('Position', [2.7, 2.3, 0.6, 0.4], 'FaceColor', [0.85 0.95 0.85], 'EdgeColor', [0.2 0.6 0.2], 'LineWidth', 1.5);
text(3.0, 2.5, 'R_M', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.1 0.5 0.1]);

line([3.3, 5.5], [2.5, 2.5], 'LineWidth', 3, 'Color', [0 0.2 0.8]);
text(4.4, 2.8, 'x km (Line 1)', 'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', [0 0.2 0.8]);

plot(5.5, 2.5, 'p', 'MarkerSize', 16, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
text(5.5, 3.2, 'Fault Point (F)', 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
line([5.5, 5.5], [2.5, 1.8], 'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
text(5.5, 1.5, 'R_f', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

line([5.5, 7.7], [2.5, 2.5], 'LineWidth', 3, 'Color', [0 0.2 0.8]);
text(6.6, 2.8, '(120 - x) km (Line 2)', 'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', [0 0.2 0.8]);

line([7.7, 7.7], [1.2, 3.8], 'LineWidth', 4, 'Color', 'k');
text(7.7, 4.1, 'Bus N', 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
line([7.7, 8.4], [2.5, 2.5], 'LineWidth', 2, 'Color', 'k');
rectangle('Position', [8.4, 2.1, 0.8, 0.8], 'Curvature', [1, 1], 'LineWidth', 2, 'EdgeColor', [0 0.447 0.741]);
text(8.8, 2.5, 'S_N', 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

text(5.0, 0.6, '230 kV, 120 km System | z_1 = 0.02 + j0.28678 \Omega/km, z_0 = 0.10645 + j0.836989 \Omega/km', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.95 0.95 0.95]);
title('Fig 1: Two-Bus Equivalent Power Transmission System with Relay R_M at Bus M', 'FontSize', 13, 'FontWeight', 'bold');
saveas(f1, 'Fig1_System_Schematic.png');
close(f1);

% Fig 2: Relay Processing Pipeline
f2 = figure('Visible', 'off', 'Position', [100, 100, 900, 340], 'Color', 'w');
axes('Position', [0.03, 0.1, 0.94, 0.8]);
hold on; axis off;
xlim([0, 12]); ylim([0, 4]);

stages = {
    {'Input Currents', '(i_a, i_b, i_c)'}, ...
    {'1 kHz Sampling', '(T_s = 1 ms)'}, ...
    {'Fault Detector', '|\Deltai_n| > 100A'}, ...
    {'1-Cycle DFT', 'Fundamental Phasors'}, ...
    {'Fortescue Sequence', '[T]^{-1} -> I_0, I_1, I_2'}, ...
    {'Angular Classifier', '(\delta_I^+ & I_0 Rules)'}
};
box_colors = {[0.92 0.92 0.92], [0.9 0.95 1], [1 0.92 0.8], [0.85 0.95 0.85], [1 0.88 0.88], [0.95 0.88 1]};

for s = 1:6
    bx = (s - 1) * 2.0 + 0.3;
    rectangle('Position', [bx, 1.2, 1.5, 1.6], 'Curvature', 0.2, 'FaceColor', box_colors{s}, 'LineWidth', 1.8);
    text(bx + 0.75, 2.0, stages{s}, 'HorizontalAlignment', 'center', 'FontSize', 9.5, 'FontWeight', 'bold', 'Interpreter', 'tex');
    if s < 6
        quiver(bx + 1.5, 2.0, 0.45, 0, 0, 'MaxHeadSize', 2, 'LineWidth', 2, 'Color', 'k');
    end
end
title('Fig 2: Digital Distance Relay Processing Pipeline & Sequence Fault Classifier (Lecture 32, 33, 35)', 'FontSize', 12, 'FontWeight', 'bold');
saveas(f2, 'Fig2_Relay_Architecture.png');
close(f2);

% Fig 3: Sequence Angular Sector Diagram
f3 = figure('Visible', 'off', 'Position', [100, 100, 600, 600], 'Color', 'w');
hold on; axis equal;
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]);

theta_c = linspace(0, 2*pi, 300);
plot(cos(theta_c), sin(theta_c), 'k--', 'LineWidth', 1.2);
plot(0, 0, 'ko', 'MarkerFaceColor', 'k');

sectors = [0, 60, 120, 180, 240, 300];
sector_labels = {'AG', 'AB / ABG', 'BG', 'BC / BCG', 'CG', 'CA / CAG'};

for s = 1:length(sectors)
    ang = sectors(s);
    rad = deg2rad(ang);
    rad_b1 = deg2rad(ang - 30);
    plot([0, 1.25*cos(rad_b1)], [0, 1.25*sin(rad_b1)], 'Color', [0.7 0.7 0.7], 'LineStyle', ':');
    quiver(0, 0, 0.95*cos(rad), 0.95*sin(rad), 0, 'LineWidth', 2.5, 'MaxHeadSize', 0.4, 'Color', [0.85 0.1 0.1]);
    text(1.35*cos(rad), 1.35*sin(rad), sprintf('%s\n(%d deg)', sector_labels{s}, ang), ...
        'HorizontalAlignment', 'center', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0.2 0.7]);
end

quiver(0, 0, 1.0, 0, 0, 'LineWidth', 3, 'LineStyle', '--', 'MaxHeadSize', 0.3, 'Color', [0 0.5 0]);
text(0.5, -0.15, 'I_1 Reference', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
title('Fig 3: Sequence Current Angular Sectors (\delta_I^+ = \angle I_2 - \angle I_1) [Lecture 37, Slide 2]', ...
    'FontSize', 11, 'FontWeight', 'bold');
saveas(f3, 'Fig3_Angular_Sector_Classification.png');
close(f3);

% Fig 4: Simulated Three-Phase Current Waveforms from Simulink
f4 = figure('Visible', 'off', 'Position', [100, 100, 900, 480], 'Color', 'w');
t_ms = (0:(size(sample_waveforms.AG, 1) - 1)) * 1.0; % 1 ms sampling = 1000 Hz

subplot(2, 1, 1);
plot(t_ms, sample_waveforms.AG, 'LineWidth', 1.5);
xline(40, 'r--', 'Fault Inception (40 ms)', 'LineWidth', 1.5, 'FontSize', 10);
grid on; ylabel('Current (A)'); xlabel('Time (ms)');
title('Simulink Acquired Currents at Relay R_M: Phase A-to-Ground (AG) Fault at 30 km (Rf = 0.5 \Omega)');
legend('i_a', 'i_b', 'i_c', 'Location', 'northeast');

subplot(2, 1, 2);
plot(t_ms, sample_waveforms.BCG, 'LineWidth', 1.5);
xline(40, 'r--', 'Fault Inception (40 ms)', 'LineWidth', 1.5, 'FontSize', 10);
grid on; ylabel('Current (A)'); xlabel('Time (ms)');
title('Simulink Acquired Currents at Relay R_M: Double Line-to-Ground (BCG) Fault at 30 km (Rf = 0.5 \Omega)');
legend('i_a', 'i_b', 'i_c', 'Location', 'northeast');

saveas(f4, 'Fig4_Waveforms_Sample.png');
close(f4);

% Fig 5: 11x11 Confusion Matrix Heatmap
f5 = figure('Visible', 'off', 'Position', [100, 100, 750, 650], 'Color', 'w');
imagesc(confusion_matrix);
colormap(flipud(summer));
colorbar;
xticks(1:n_classes); xticklabels(classes);
yticks(1:n_classes); yticklabels(classes);
set(gca, 'TickLabelInterpreter', 'none');
xlabel('Predicted Class', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('True Class', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Fig 5: Simulink Verification Confusion Matrix (Accuracy: %.2f%% across %d simulated cases)', ...
    accuracy, total_cases), 'FontSize', 11, 'FontWeight', 'bold');

for i = 1:n_classes
    for j = 1:n_classes
        val = confusion_matrix(i, j);
        txt_color = 'k';
        if val > 0.5 * max(confusion_matrix(:)), txt_color = 'w'; end
        text(j, i, sprintf('%d', val), 'HorizontalAlignment', 'center', ...
            'FontSize', 11, 'FontWeight', 'bold', 'Color', txt_color);
    end
end
saveas(f5, 'Fig5_Confusion_Matrix.png');
close(f5);

fprintf('All verification figures saved successfully.\n');
fprintf('Verification completed successfully.\n');
close_system(modelName, 0);
