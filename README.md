# ⚡ Autonomous Fault Classification in Power Networks

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%20%7C%20R2022b%2B-blue.svg?logo=mathworks)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Simscape%20Electrical-orange.svg?logo=mathworks)](https://www.mathworks.com/products/simscape.html)
[![Accuracy](https://img.shields.io/badge/Accuracy-100.00%25%20(42%2F42)-brightgreen.svg)]()
[![Course](https://img.shields.io/badge/EE315-Power%20Systems-red.svg)]()
[![IIT Indore](https://img.shields.io/badge/IIT%20Indore-Electrical%20Engineering-darkblue.svg)]()

> **Autonomous Digital Distance Relay Fault Classification Engine** built strictly upon fundamental power system symmetrical components, cycle-to-cycle discrete disturbance detection, 1-cycle discrete Fourier transform (DFT), and sequence angular sector mapping. Verified against dynamic high-fidelity Simscape physical power system simulations.

---

## 📑 Table of Contents
* [🌟 Highlights & Key Features](#-highlights--key-features)
* [🎯 Problem Statement](#-problem-statement)
* [🧠 Methodology & Signal Processing Pipeline](#-methodology--signal-processing-pipeline)
* [📂 Repository Structure & File Guide](#-repository-structure--file-guide)
* [🚀 How to Run Locally](#-how-to-run-locally)
* [⚡ Power Network Simulation Model](#-power-network-simulation-model)
* [📊 Verification Results & Benchmark (42/42)](#-verification-results--benchmark-4242)
* [🛡️ Defensive Guards & Load Invariance](#️-defensive-guards--load-invariance)
* [📜 References](#-references)

---

## 🌟 Highlights & Key Features

* 🚀 **100% Course-Grounded:** Built entirely from fundamental electrical engineering principles (Fortescue sequence components, Fourier phasor estimation, and sequence angular sectors). **Zero black-box ML or neural networks.**
* 🔄 **Load-Flow Invariance:** Uses cycle-to-cycle pre-fault subtraction ($\Delta i = i_{\text{post}} - i_{\text{pre}}$) to strip away pre-fault load current ($I_{1,\text{load}}$). Retains 100% classification stability regardless of power flow direction or load angle ($\pm 45^\circ$).
* 🛡️ **Defensive Numerical Noise Guard:** Prevents false angle calculations on balanced three-phase (`ABC`) and normal steady-state (`NO_FAULT`) conditions where negative-sequence current $I_2 \approx 0$.
* ⚡ **Full Simscape Verification:** Tested programmatically across **42 dynamic Simulink simulations** covering 11 operating classes, near/far fault locations ($25\%$ and $66.7\%$ of line), bolted and high-resistance faults, and forward/reverse load flows.

---

## 🎯 Problem Statement

Modern numerical distance relays (IEEE Device 21) require autonomous and rapid fault classification within 1 to 2 cycles of fault inception to:
1. **Select Correct Impedance Measuring Elements:** Choose between line-to-ground loops ($Z_a = \frac{V_a}{I_a + K_0 I_0}$) and line-to-line loops ($Z_{ab} = \frac{V_a - V_b}{I_a - I_b}$).
2. **Enable Single-Pole Autoreclosing (SPAR):** Single line-to-ground faults account for $70\text{--}80\%$ of overhead transmission incidents. Classifying single-phase faults permits tripping only the faulted phase while keeping healthy phases energised for system stability.

### Target Classification Classes (11 Classes)
| Class Category | Operating / Fault Classes |
| :--- | :--- |
| **Normal Condition** | `NO_FAULT` (Forward load flow & Reverse power flow) |
| **Single Line-to-Ground (SLG)** | `AG`, `BG`, `CG` |
| **Line-to-Line (LL)** | `AB`, `BC`, `CA` |
| **Double Line-to-Ground (LLG)** | `ABG`, `BCG`, `CAG` |
| **Three-Phase Symmetrical (LLL)**| `ABC` |

---

## 🧠 Methodology & Signal Processing Pipeline

The relay algorithm processes discrete 3-phase instantaneous currents $i_{abc}(n)$ through five sequential stages:

<p align="center">
  <img src="Fig2_Relay_Architecture.png" alt="Digital Relay Pipeline" width="850">
  <br>
  <em>Figure 1: Digital Distance Relay Processing Pipeline implemented in <code>fault_classifier.m</code>.</em>
</p>

### 1. Cycle-to-Cycle Disturbance Detection
Samples are compared with the previous cycle ($N = 20$ samples at $f_s = 1000\text{ Hz}$, $f = 50\text{ Hz}$):
$$\Delta i_k(n) = \lvert i_k(n) - i_k(n-N) \rvert$$
* If $\max_{k} \Delta i_k(n) \le 100\text{ A}$ $\implies$ **`NO_FAULT`** (Normal steady-state).
* If $\max_{k} \Delta i_k(n) > 100\text{ A}$ $\implies$ **Fault Inception Flagged** at sample $n_{\text{det}}$.

### 2. Pre-Fault Cycle Subtraction (Pure Fault Incremental Currents)
To cancel out positive-sequence load contamination ($I_{1,\text{load}}$), the pre-fault cycle is subtracted point-by-point from the post-fault cycle:
$$\Delta i_k(n) = i_k(n) - i_k(n - N_{\text{cycle}})$$

### 3. Full-Cycle Discrete Fourier Transform (DFT)
Fundamental frequency current phasors $\dot{I}_a, \dot{I}_b, \dot{I}_c$ are computed over one full cycle ($N = 20$):
$$\dot{I} = \frac{\sqrt{2}}{N} \sum_{n=0}^{N-1} \Delta i(n) \, e^{-j \frac{2\pi n}{N}}$$

### 4. Symmetrical Components Transformation (Fortescue)
Sequence components are extracted via the classical Fortescue transformation:
$$\begin{bmatrix} \dot{I}_0 \\ \dot{I}_1 \\ \dot{I}_2 \end{bmatrix} = \frac{1}{3} \begin{bmatrix} 1 & 1 & 1 \\ 1 & a & a^2 \\ 1 & a^2 & a \end{bmatrix} \begin{bmatrix} \dot{I}_a \\ \dot{I}_b \\ \dot{I}_c \end{bmatrix}, \quad a = e^{j 120^\circ}$$

### 5. Ground Discrimination & Angular Sector Mapping
* **Balanced Three-Phase Check:** If $|I_2|/|I_1| < 0.08$ and $|I_0|/|I_1| < 0.08$ $\implies$ **`ABC` Fault** (Balanced; $\delta_I = \text{NaN}$).
* **Ground Detection:** Ground involvement is flagged if:
  $$\frac{\lvert I_0 \rvert}{\lvert I_1 \rvert} > 0.05 \quad \text{AND} \quad \lvert I_0 \rvert > 20\text{ A}$$
* **Sequence Current Angle Difference:**
  $$\delta_I^+ = \left( \angle \dot{I}_2 - \angle \dot{I}_1 \right) \pmod{360^\circ}$$

<p align="center">
  <img src="Fig3_Angular_Sector_Classification.png" alt="Angular Sector Classification" width="550">
  <br>
  <em>Figure 2: Sequence Current Angular Sectors ($\delta_I^+ = \angle I_2 - \angle I_1$) for fault phase identification.</em>
</p>

| Ground Involved? | Angular Sector Range | Classified Fault Type |
| :---: | :---: | :---: |
| **Yes** | $330^\circ \le \delta_I^+ < 30^\circ$ | **`AG`** |
| **Yes** | $30^\circ \le \delta_I^+ < 90^\circ$ | **`ABG`** |
| **Yes** | $90^\circ \le \delta_I^+ < 150^\circ$ | **`BG`** |
| **Yes** | $150^\circ \le \delta_I^+ < 210^\circ$ | **`BCG`** |
| **Yes** | $210^\circ \le \delta_I^+ < 270^\circ$ | **`CG`** |
| **Yes** | $270^\circ \le \delta_I^+ < 330^\circ$ | **`CAG`** |
| **No** | $30^\circ \le \delta_I^+ < 90^\circ$ | **`AB`** |
| **No** | $150^\circ \le \delta_I^+ < 210^\circ$ | **`BC`** |
| **No** | $270^\circ \le \delta_I^+ < 330^\circ$ | **`CA`** |

---

## 📂 Repository Structure & File Guide

```
PowerSystem_A12/
├── 📄 README.md                            # Comprehensive interactive repository guide
├── ⚙️ fault_classifier.m                   # Standalone relay classification function
├── 🧪 run_project_verification.m           # Automated 42-run test harness & confusion matrix
├── 🏗️ build_simulink_model.m               # Script to build power_network_model.slx from scratch
├── ⚡ power_network_model.slx              # 230 kV, 120 km Two-Bus Simscape physical grid model
├── 🖼️ Fig1_System_Schematic.png            # Two-Bus power network schematic
├── 🖼️ Fig2_Relay_Architecture.png          # Relay processing pipeline diagram
├── 🖼️ Fig3_Angular_Sector_Classification.png # Symmetrical component angular sector circle
├── 🖼️ Fig4_Waveforms_Sample.png            # Three-phase current waveforms from Simulink
└── 🖼️ Fig5_Confusion_Matrix.png            # 11x11 confusion matrix heatmap (100% accuracy)
```

### 🔍 Detailed File Significance

| File | Type | Primary Purpose | How It Works |
| :--- | :---: | :--- | :--- |
| **`fault_classifier.m`** | MATLAB Function | **Core Relay Brain:** Evaluates raw current matrix `I_abc` and returns fault type string. | Takes instantaneous currents, applies cycle-to-cycle detection, extracts pure fault cycle, executes DFT, Fortescue transform, and maps to angular sectors. |
| **`power_network_model.slx`** | Simulink Model | **Physical Data Source:** Simscape Specialized Power Systems 230 kV grid. | Contains local Bus M, remote Bus N, two PI section line segments, Three-Phase Fault block, and 1 kHz CT current loggers. |
| **`run_project_verification.m`** | MATLAB Script | **Automated Test Suite:** Runs all 42 Simulink cases and generates benchmark plots. | Automatically updates Simulink block parameters, executes simulations, feeds data to `fault_classifier`, logs performance, and exports Figures 1–5. |
| **`build_simulink_model.m`** | MATLAB Script | **Model Rebuilder:** Programmatically recreates `power_network_model.slx` from code. | Uses Simulink API (`add_block`, `add_line`, `set_param`) to construct the complete physical grid model from scratch with zero manual clicks. |

---

## 🚀 How to Run Locally

### Prerequisites
* **MATLAB:** R2022b or later (Tested on R2024b).
* **Toolboxes:** Simulink, Simscape, and Simscape Electrical (Specialized Power Systems / `powerlib`).

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/Krish-Prasad09/PowerSystem_A12.git
cd PowerSystem_A12
```

### 2️⃣ Run Full Automated Verification in MATLAB
Open MATLAB, navigate to the folder, and run:
```matlab
% Run all 42 Simulink simulation cases and generate confusion matrix
run_project_verification
```
*What happens:* MATLAB will simulate 42 fault/load scenarios across the line, evaluate them in real time, print a formatted progress table to the command window, compute the $11 \times 11$ confusion matrix, and save publication-grade plots.

### 3️⃣ Test a Single Custom Case in MATLAB
```matlab
% 1. Open the Simulink model
open_system('power_network_model');

% 2. Set custom fault parameters (e.g., AG fault at 50 km, Rf = 5 ohms)
set_param('power_network_model/Line1', 'Length', '50');
set_param('power_network_model/Line2', 'Length', '70');
set_param('power_network_model/Fault', 'FaultType', '100'); % Phase A to ground
set_param('power_network_model/Fault', 'FaultResistance', '5');

% 3. Run simulation
simOut = sim('power_network_model', 'StopTime', '0.1');

% 4. Classify current samples from Bus M relay CT
I_abc = simOut.I_relay.Data;
t_vec = simOut.I_relay.Time;
[faultType, I_phasors, delta_I, debugInfo] = fault_classifier(I_abc, 1000);

fprintf('🎯 Detected Fault Type: %s (Sector Angle: %.2f°)\n', faultType, delta_I);
```

---

## ⚡ Power Network Simulation Model

The physical grid model is built in Simscape Specialized Power Systems:
* **System Voltage:** $230\text{ kV}$ line-to-line, $50\text{ Hz}$, Three-Phase.
* **Line Length:** $120\text{ km}$, modelled with lumped $\pi$-sections.
* **Line Parameters (Lecture 35):**
  * $r_1 = 0.02\ \Omega/\text{km}, \quad l_1 = 0.91285\text{ mH/km} \implies z_1 = 0.02 + j0.28678\ \Omega/\text{km}$
  * $r_0 = 0.10645\ \Omega/\text{km}, \quad l_0 = 2.6642\text{ mH/km} \implies z_0 = 0.10645 + j0.836989\ \Omega/\text{km}$
  * Zero-sequence compensation factor: $K_0 = \frac{z_0 - z_1}{3 z_1} = 0.6409 \angle -0.73^\circ$
  * Shunt Capacitances: $C_1 = 14.0\text{ nF/km}, \quad C_0 = 7.75\text{ nF/km}$ ($v_1 \approx 280,000\text{ km/s} < c$).
* **Sources:**
  * Local Bus M: $230\text{ kV} \angle 0^\circ, \quad Z_{SM} = 0.5 + j5.0\ \Omega$
  * Remote Bus N: $230\text{ kV} \angle -10^\circ, \quad Z_{SN} = 0.5 + j5.0\ \Omega$

<p align="center">
  <img src="Fig1_System_Schematic.png" alt="Two-Bus System Schematic" width="800">
  <br>
  <em>Figure 3: Two-Bus 230 kV, 120 km Power Network modeled in Simulink.</em>
</p>

<p align="center">
  <img src="Fig4_Waveforms_Sample.png" alt="Simulink Waveforms" width="800">
  <br>
  <em>Figure 4: Instantaneous CT currents logged at Bus M for Phase A-to-ground (AG) and Double Line-to-ground (BCG) faults.</em>
</p>

---

## 📊 Verification Results & Benchmark (42/42)

The automated test suite evaluates **42 independent scenarios** directly simulated through `power_network_model.slx`:
* **Fault Locations:** Near Bus M ($30\text{ km}$, $25\%$ of line) and Far End ($80\text{ km}$, $66.7\%$ of line).
* **Fault Resistances:** Bolted ($R_f = 0.5\ \Omega$) and Moderate Ground ($R_f = 10.0\ \Omega$).
* **Load Flow Conditions:** Forward power flow ($\delta_N = -10^\circ$) and Reverse power flow ($\delta_N = +5^\circ$).

### Summary Performance
| Metric | Value |
| :--- | :---: |
| **Total Simulated Cases** | **42** |
| **Correctly Classified** | **42** |
| **Misclassifications** | **0** |
| **Overall Accuracy** | **100.00%** |
| **Decision Speed** | **1.0 to 1.5 cycles post-detection** |

<p align="center">
  <img src="Fig5_Confusion_Matrix.png" alt="Confusion Matrix" width="750">
  <br>
  <em>Figure 5: 11x11 Confusion Matrix heatmap demonstrating 100% classification across all 42 Simulink test runs.</em>
</p>

---

## 🛡️ Defensive Guards & Load Invariance

### 1. Immunity to Load Flow Direction & Load Angle
Under heavy load or reversed power transfer, positive-sequence load current $I_{1,\text{load}}$ tilts the angle of total current $\dot{I}_1$. In conventional classifiers, this rotation pushes borderline cases across sector boundaries. 
By calculating **pure fault component currents** ($\Delta i = i_{\text{post}} - i_{\text{pre}}$), $I_{1,\text{load}}$ cancels to zero:
$$\Delta \dot{I}_1 = (\dot{I}_{1,\text{fault}} + \dot{I}_{1,\text{load}}) - \dot{I}_{1,\text{load}} = \dot{I}_{1,\text{fault}}$$
*Independent stress testing confirmed 100% stability across load angles from $-45^\circ$ to $+45^\circ$ and frequency variations from $49.5\text{ Hz}$ to $50.5\text{ Hz}$.*

### 2. Numerical Noise Guarding for Balanced Faults
For balanced three-phase faults (`ABC`) and normal operation (`NO_FAULT`), negative-sequence current is theoretically zero ($I_2 = 0$). In numerical simulations, computing $\angle \dot{I}_2$ evaluates random round-off noise. The classifier guards against this by checking sequence ratios ($|I_2|/|I_1| < 0.08$ and $|I_0|/|I_1| < 0.08$) and setting $\delta_I = \text{NaN}$, ensuring unphysical angles are never evaluated.

---

## 📜 References

1. **Course Lecture Slides (EE 315: Power Systems, IIT Indore):**
   * *Lecture 26:* Symmetrical Components & Fortescue Transformation Matrix.
   * *Lecture 27:* Sequence Networks & Zero-Sequence Ground Path Analysis.
   * *Lecture 32 & 35:* Transmission Line Distance Relaying Principles & Line Parameter Calculations.
   * *Lecture 33:* Numerical Relaying Algorithms, Discrete Cycle-to-Cycle Disturbance Detection, & 1-Cycle DFT.
   * *Lecture 37:* Sequence Current Angular Sectors ($\delta_I^+ = \angle I_2 - \angle I_1$) for Fast Fault Classification.
2. **Standard Reference Literature:**
   * A. G. Phadke and J. S. Thorp, *Computer Relaying for Power Systems*, John Wiley & Sons, 2nd ed., 2009.
   * P. M. Anderson, *Analysis of Faulted Power Systems*, IEEE Press Power Engineering Series, 1995.

---

<p align="center">
  <strong>Department of Electrical Engineering &bull; Indian Institute of Technology Indore</strong><br>
  <em>Academic Year 2026-2027 &bull; EE 315: Power Systems</em>
</p>
