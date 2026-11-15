# Trim, Thrust Required, and Power Required Analysis of a Cropped Delta Wing UAV

A computational and analytical flight mechanics project evaluating the longitudinal pitch trim, steady level-flight equilibrium, propulsion requirements, and aerodynamic efficiency of a **3.5 kg wing-alone cropped delta reflex UAV** across an angle-of-attack sweep from α = 0° to 12° (Δα = 0.5°).

## Project Overview

Tailless cropped delta UAVs rely on trailing-edge elevons (operating symmetrically as an elevator, δe) and reflex airfoil camber (Cm0 > 0) to maintain longitudinal static stability and pitch trim without a horizontal stabilizer. Because elevon deflection directly alters main-wing camber and lift, pitch trim (Cm = 0) is coupled to total lift (CL), level-flight airspeed (V), induced drag (CDi), thrust required (Tr), and power required (Pr).

This project includes the modular MATLAB solver, dynamic Excel workbook, Google Workspace deliverables, and full engineering report covering all three assignment phases:

1. **Phase 1 — Preprocessing & Iterative Trim Solver:** Computes geometric parameters (λ, S, AR, k) and sweeps α = 0° to 12° (0.5° step) to solve δe,trim, CL,trim, V, CD, Tr, and Pr.
2. **Phase 2 — Performance Metrics:** Evaluates range efficiency (CL/CD) and endurance parameter (CL^(3/2)/CD), benchmarking discrete extrema against exact closed-form parabolic drag polar theory.
3. **Phase 3 — Visualization & Reporting:** Generates all eight required plots with SI units and documents methodology, results, technical challenges, and method trade-offs.

## UAV Configuration & Given Parameters

| Category | Parameter | Symbol | Value | Units |
| :--- | :--- | :--- | :--- | :--- |
| **Geometry** | Root Chord | Cr | 0.90 | m |
| **Geometry** | Tip Chord | Ct | 0.15 | m |
| **Geometry** | Wingspan | b | 1.50 | m |
| **Geometry** | Oswald Efficiency Factor | e | 0.89 | — |
| **Aerodynamics** | Zero-Lift Parasite Drag | CD0 | 0.030 | — |
| **Aerodynamics** | Zero-AoA Lift Coefficient | CL0 | 0.000 | — |
| **Aerodynamics** | Zero-Lift Pitching Moment | Cm0 | +0.010 | — |
| **Aerodynamics** | Wing Lift Curve Slope | CLα | 2.920 | rad⁻¹ |
| **Aerodynamics** | Elevator Lift Derivative | CLδe | 0.265 | rad⁻¹ |
| **Aerodynamics** | Static Stability Derivative | Cmα | −0.292 | rad⁻¹ |
| **Aerodynamics** | Elevator Pitch Authority | Cmδe | −0.400 | rad⁻¹ |
| **Flight Conditions** | UAV Mass | m | 3.50 | kg |
| **Flight Conditions** | Gravity / Weight | g / W | 10.0 / 35.0 | m/s² / N |
| **Flight Conditions** | Sea-Level Air Density | ρ | 1.225 | kg/m³ |

### Preprocessing Results

* **Taper Ratio (λ = Ct / Cr):** 0.1667 (1/6)
* **Planform Area (S = (b/2) × Cr × (1 + λ)):** 0.7875 m²
* **Aspect Ratio (AR = b² / S):** 2.8571 (20/7)
* **Induced Drag Factor (k = 1 / (π × e × AR)):** 0.12516
* **Effective Trimmed Lift Slope (CLα,trim = CLα − CLδe × (Cmα / Cmδe)):** 2.7266 rad⁻¹

## Project Files & Deliverables

| File / Resource | Format | Description |
| :--- | :--- | :--- |
| `uav_trim_performance.m` | MATLAB Script | Modular, well-commented 3-phase solver that computes preprocessing values, runs the 25-point sweep, prints the formatted console table and optima comparison, and generates 2 multi-panel figures (all 8 plots). |
| `UAV_Trim_Performance_Analysis.xlsx` | Excel Workbook | Formula-driven spreadsheet (`In` and `Sweep` tabs) with live equilibrium formulas, discrete vs. analytical optima benchmarks, and embedded charts. |
| `UAV_Trim_Performance_Project_Report.pdf` | PDF Report | Compiled LaTeX engineering report with complete derivations, MATLAB listing, 25-point table, and 4 high-DPI figure panels. |
| **Google Doc Report** | Google Docs | Fully editable project report including all tables, MATLAB code + Command Window output, linked Drive plots, and technical challenges/methods analysis. |
| **Google Sheet Tables** | Google Sheets | Companion 3-tab spreadsheet (`Table 1 - Parameters`, `Table 2 - Trim Analysis`, `Table 3 - Theory Comparison`) with live formulas. |

## Key Numerical Results

| Performance Metric | Discrete Sweep Result | Exact Analytical Theory | Physical Significance |
| :--- | :--- | :--- | :--- |
| **Self-Trimmed AoA (δe,trim = 0°)** | α = 2.0° (δe = −0.028°) | α = 1.9622° | Neutral-elevon hands-off cruise point (V ≈ 26.95 m/s). |
| **Max Range / Min Thrust (Tr,min)** | 4.2901 N at α = 10.0° (V = 12.26 m/s) | 4.2897 N at α = 10.15° (V = 12.17 m/s) | Peak (CL/CD)max = 8.1583 (0.01% from analytical 8.1592). |
| **Max Endurance / Min Power (Pr,min)** | 48.74 W at α = 12.0° (V = 11.21 m/s) | 45.82 W at α = 17.68° (V = 9.25 m/s) | Peak CL^(3/2)/CD = 6.1174 within the 0°–12° sweep (94.0% of unconstrained peak). |

## How to Run

### MATLAB

1. Open MATLAB and set your working directory to the folder containing `uav_trim_performance.m`.
2. Run the script:

   ```matlab
   uav_trim_performance
   ```

3. The script prints the preprocessing summary, the 25-row steady level-flight performance table, and the discrete-vs.-analytical optima comparison to the Command Window, and opens two 2×2 figure windows displaying all 8 plots:
   * **Figure 1:** Trim AoA vs. Velocity, Trim Elevator Deflection vs. AoA, Trimmed Lift Coefficient vs. AoA, Drag Coefficient vs. AoA.
   * **Figure 2:** Thrust Required vs. Velocity, Power Required vs. Velocity, CL/CD vs. AoA, CL^(3/2)/CD vs. AoA.

### Excel / Google Sheets

* Open `UAV_Trim_Performance_Analysis.xlsx` (or the companion Google Sheet) and modify any input in the **Parameters** tab to automatically recalculate the entire 25-point sweep and update the embedded performance charts.

## Modeling Assumptions & Scope Notes

* **Steady, Unaccelerated Level Flight:** γ = 0°, φ = 0°, L = W = 35.0 N, and Tr = D.
* **Linear Aerodynamics & Zero Baseline Camber Lift:** Assumes CL0 = 0 and linear derivatives across α = 0° to 12° prior to high-AoA vortex breakdown.
* **Radian Consistency:** Converts α to radians before evaluating Cm = 0 and CL, then converts δe,trim back to degrees for reporting.
* **Practical Envelope Windowing:** α = 0° produces a small reflex-trim lift (CL,trim = 0.006625) yielding an asymptotic velocity of 104.66 m/s; velocity-domain plots focus on α ≥ 0.5° (V ≤ 48.84 m/s) for clear visualization of the thrust and power buckets.