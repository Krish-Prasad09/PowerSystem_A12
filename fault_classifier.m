function [faultType, info] = fault_classifier(I_abc, fs)
% FAULT_CLASSIFIER Digital relay fault classification using sampled currents.
%
% Implements the fault classification technique based on EE 315 course material:
%   1. Cycle-to-cycle current difference for fault detection (Lecture 33, Slide 3).
%   2. 1-cycle fundamental DFT phasor estimation (Lecture 33, Slides 7-10).
%   3. Fortescue symmetrical sequence transformation (Lecture 26, Slide 4).
%   4. Sequence current angular sector logic (delta_I = angle(I2) - angle(I1))
%      and zero-sequence ground discrimination (Lecture 37, Slide 2).
%
% Inputs:
%   I_abc - (N x 3) matrix of sampled phase currents [ia, ib, ic] in Amperes.
%   fs    - Sampling frequency in Hz (default = 1000 Hz, 20 samples/cycle at 50 Hz).
%
% Outputs:
%   faultType - String: 'NO_FAULT', 'AG', 'BG', 'CG', 'AB', 'BC', 'CA',
%               'ABG', 'BCG', 'CAG', 'ABC', or 'UNCERTAIN'.
%   info      - Diagnostic struct with sequence currents, ratios, and angles.

    if nargin < 2 || isempty(fs)
        fs = 1000; % 1000 Hz sampling (20 samples per 50 Hz fundamental cycle)
    end

    f0 = 50; % Fundamental frequency (Hz)
    N_cycle = round(fs / f0);
    numSamples = size(I_abc, 1);

    % Default output struct
    info.faultDetected = false;
    info.I0_mag = 0;
    info.I1_mag = 0;
    info.I2_mag = 0;
    info.I0_over_I1 = 0;
    info.I2_over_I1 = 0;
    info.delta_deg = NaN;

    % -------------------------------------------------------------------------
    % Step 1: Fault Disturbance Detection via Cycle-to-Cycle Comparison (Lecture 33)
    % delta_i_k(n) = |i_k(n) - i_k(n - N_cycle)|
    % -------------------------------------------------------------------------
    det_threshold = 100; % Amperes (minimum disturbance threshold above normal load)
    fault_idx = 0;

    for n = (N_cycle + 1):numSamples
        d_ia = abs(I_abc(n, 1) - I_abc(n - N_cycle, 1));
        d_ib = abs(I_abc(n, 2) - I_abc(n - N_cycle, 2));
        d_ic = abs(I_abc(n, 3) - I_abc(n - N_cycle, 3));

        if (d_ia > det_threshold) || (d_ib > det_threshold) || (d_ic > det_threshold)
            fault_idx = n;
            info.faultDetected = true;
            break;
        end
    end

    % If no fault disturbance is detected, record is normal steady-state operation
    if ~info.faultDetected
        faultType = 'NO_FAULT';
        return;
    end

    % -------------------------------------------------------------------------
    % Step 2: Extract 1-Cycle Window for Fundamental Phasor Estimation (Lecture 33)
    % Allow 1 fundamental cycle after detection for transient settling
    % -------------------------------------------------------------------------
    win_start = fault_idx + N_cycle;
    win_end   = win_start + N_cycle - 1;

    if win_end > numSamples
        win_end = numSamples;
        win_start = win_end - N_cycle + 1;
    end

    if win_start < 1
        faultType = 'UNCERTAIN';
        return;
    end

    % Subtract pre-fault cycle (superimposed / fault-component currents)
    % Directly derived from Lecture 33, Slide 3 cycle-to-cycle principle:
    % Delta_i(n) = i(n) - i(n - k*N). Pre-fault load is balanced, so subtracting
    % the pre-fault cycle cancels out load current contamination from I1,
    % making delta_I+ strictly invariant to load angle and power flow direction.
    pre_start = mod(win_start - 1, N_cycle) + 1;
    pre_end   = pre_start + N_cycle - 1;

    if (pre_end < fault_idx) && (pre_end <= numSamples)
        I_eval = I_abc(win_start:win_end, :) - I_abc(pre_start:pre_end, :);
    else
        I_eval = I_abc(win_start:win_end, :);
    end

    % 1-Cycle Discrete Fourier Transform (DFT) (Lecture 33, Slide 7)
    % I_k = (sqrt(2) / N) * sum_{n=0}^{N-1} i_k(n) * exp(-j * 2*pi*n / N)
    n_vec = (0:(N_cycle - 1))';
    dft_kernel = exp(-1j * 2 * pi * n_vec / N_cycle);

    Ia = (sqrt(2) / N_cycle) * sum(I_eval(:, 1) .* dft_kernel);
    Ib = (sqrt(2) / N_cycle) * sum(I_eval(:, 2) .* dft_kernel);
    Ic = (sqrt(2) / N_cycle) * sum(I_eval(:, 3) .* dft_kernel);

    % -------------------------------------------------------------------------
    % Step 3: Symmetrical Sequence Transformation (Lecture 26, Slide 4)
    % [I0; I1; I2] = (1/3) * [1 1 1; 1 a a^2; 1 a^2 a] * [Ia; Ib; Ic]
    % -------------------------------------------------------------------------
    a = exp(1j * 2 * pi / 3);
    T_inv = (1/3) * [1, 1,   1;
                     1, a,   a^2;
                     1, a^2, a];

    I_seq = T_inv * [Ia; Ib; Ic];
    I0 = I_seq(1);
    I1 = I_seq(2);
    I2 = I_seq(3);

    I0_mag = abs(I0);
    I1_mag = abs(I1);
    I2_mag = abs(I2);

    info.I0_mag = I0_mag;
    info.I1_mag = I1_mag;
    info.I2_mag = I2_mag;
    info.I0_over_I1 = I0_mag / (I1_mag + 1e-6);
    info.I2_over_I1 = I2_mag / (I1_mag + 1e-6);

    % -------------------------------------------------------------------------
    % Step 4: Decision Rule
    % -------------------------------------------------------------------------
    % Rule 4A: Symmetrical Three-Phase Fault (ABC)
    % In balanced 3-phase faults, I2 ~ 0 and I0 ~ 0, while I1 is large.
    % When I2 is near zero, its phase angle is purely numerical noise and
    % must NOT be evaluated for angular classification.
    if (info.I2_over_I1 < 0.08) && (info.I0_over_I1 < 0.08) && (I1_mag > 1000)
        faultType = 'ABC';
        info.delta_deg = NaN; % Explicitly mark angle as non-meaningful
        return;
    end

    % For unbalanced faults, verify that I2 is sufficiently large for meaningful phase angle
    if (info.I2_over_I1 < 0.08)
        % Negative sequence current is too small to make a reliable unbalanced decision
        faultType = 'UNCERTAIN';
        return;
    end

    % Rule 4B: Ground Involvement Discrimination (Lectures 26, 27)
    % Zero-sequence current exists if and only if ground path is involved.
    % Threshold: I0/I1 > 0.05 AND |I0| > 20 A
    is_ground_fault = (info.I0_over_I1 > 0.05) && (I0_mag > 20.0);

    % Rule 4C: Sequence Current Angular Sectors (Lecture 37, Slide 2)
    % delta_I = (angle(I2) - angle(I1)) mod 360 deg
    delta_deg = mod(rad2deg(angle(I2) - angle(I1)), 360);
    info.delta_deg = delta_deg;

    % Partition the 360 deg plane into 6 symmetrical 60-deg sectors:
    if (delta_deg >= 330 || delta_deg < 30)
        % 0 deg sector: AG fault (I1F = I2F = I0F, delta_I ~ 0 deg)
        if is_ground_fault
            faultType = 'AG';
        else
            faultType = 'UNCERTAIN';
        end

    elseif (delta_deg >= 30 && delta_deg < 90)
        % 60 deg sector: AB (ungrounded) or ABG (grounded)
        if is_ground_fault
            faultType = 'ABG';
        else
            faultType = 'AB';
        end

    elseif (delta_deg >= 90 && delta_deg < 150)
        % 120 deg sector: BG fault
        if is_ground_fault
            faultType = 'BG';
        else
            faultType = 'UNCERTAIN';
        end

    elseif (delta_deg >= 150 && delta_deg < 210)
        % 180 deg sector: BC (ungrounded) or BCG (grounded) (I1F = -I2F, delta_I ~ 180 deg)
        if is_ground_fault
            faultType = 'BCG';
        else
            faultType = 'BC';
        end

    elseif (delta_deg >= 210 && delta_deg < 270)
        % 240 deg sector: CG fault
        if is_ground_fault
            faultType = 'CG';
        else
            faultType = 'UNCERTAIN';
        end

    elseif (delta_deg >= 270 && delta_deg < 330)
        % 300 deg sector: CA (ungrounded) or CAG (grounded)
        if is_ground_fault
            faultType = 'CAG';
        else
            faultType = 'CA';
        end

    else
        faultType = 'UNCERTAIN';
    end
end
