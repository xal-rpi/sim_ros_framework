# Week 7 — Friction Limits + Tire Force Modeling (Homework)

**Due:** **03/11**

**Theme:** generate high-quality tire data, estimate tire forces from vehicle dynamics, and fit tire-force models that capture **linear cornering stiffness** and **friction-limited saturation**.

This handout is intentionally less step-by-step than Weeks 3–6. For launch/logging workflow, topic discovery, and basic plotting/data-loading utilities, refer to prior labs.

---

## Goals

You will:
- Use the **same vehicle as Week 5–6** in the **techground** environment.
- Collect data suitable for fitting/validating estimated tire forces:
  - **$F_{yf}$** (front lateral)
  - **$F_{xr}$** (rear longitudinal)
  - **$F_{yr}$** (rear lateral)
- Fit a **Fiala** tire model:
  - **Front:** pure slip (lateral-only)
  - **Rear:** combined slip (longitudinal + lateral)
  - Use the **exact model expressions from class notes**.

**Graduate add-on:** Fit a **Pacejka** model for the **front** tire (pure slip only), overlay on data, and report the identified parameters.

---

## Vehicle + scenario

Pick the same scenario config (vehicle selection) you used for Week 5–6:
- [src/bng_xal/bng_bringup/config/scenarios/techground_sbr.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_sbr.yaml)
- [src/bng_xal/bng_bringup/config/scenarios/techground_sunburst2.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_sunburst2.yaml)
- [src/bng_xal/bng_bringup/config/scenarios/techground_etk800.yaml](../../src/bng_xal/bng_bringup/config/scenarios/techground_etk800.yaml)

Use the corresponding vehicle parameter YAML (mass, yaw inertia, CG-to-axle distances). These are in:
- [src/bng_xal/bng_bringup/config/vehicles/](../../src/bng_xal/bng_bringup/config/vehicles/)

---

## Part 1 — Data collection (friction limits + usable masking)

### What data you need

You need enough data to cover:

1) **Small-slip linear regime**
- Front and rear slip angles concentrated in approximately **−0.2° to +0.2°**.
- Purpose: estimate **cornering stiffness** reliably.

2) **High-slip / saturation regime**
- Include **at least one run** that reaches **clearly larger slip angles** (−0.6° to +0.6) so the plots show **force saturation**.
- Purpose: estimate **friction coefficient** and identify **slip limits** for the Fiala fit.

3) **Rear combined slip (needed for $F_{xr}$ + $F_{yr}$)**
- Include segments where the rear tire is generating **meaningful longitudinal force** while also cornering.
- How you achieve that is up to you (refer to class guidance). The key is: you must end up with *usable* combined-slip data after masking.


### Masking requirements (apply in your offline analysis)

You must apply a **clear masking strategy** (as discussed in class) to isolate data suitable for:
- linear stiffness estimation (small-slip)
- saturation/friction identification (high-slip)
- combined-slip rear fitting

Your writeup must include:
- the masking rules you used (thresholds/conditions)
- a short justification for why those rules isolate the right regimes

---

## Part 2 — Acceleration consistency check (raw vs derived)

Create and discuss plots comparing:
- the logged longitudinal/lateral acceleration signal (accel_x, accel_y) from your dataset
- a derived longitudinal/lateral acceleration estimate computed from your state data (vel_x, vel_y, yaw_rate) using the kinematic relationships from class notes

Deliverables:
- one figure showing all traces over time for a representative segment
- a short discussion of discrepancies (noise, delay, bias, transients)

---

## Part 3 — Estimate $F_{yf}$, $F_{xr}$, $F_{yr}$ from vehicle dynamics

Using the approach from class notes:
- set up the per-sample system that lets you estimate **$F_{yf}$, $F_{xr}$, $F_{yr}$** from measured states/derivatives
- solve for the forces on your **masked** dataset
- verify your solution is numerically meaningful on the masked data (e.g., not singular / not ill-conditioned)

Deliverables:
- a short writeup showing (from the class-notes expression for $A$) why the matrix $A$ is **invertible** under the operating assumptions for this lab.
- scatter plots:
  - estimated lateral forces vs corresponding slip angles
  - estimated rear longitudinal force vs rear longitudinal slip proxy you used

---

## Part 4 — Fit tire models (Fiala; Pacejka for grads)

### A. Empirical characteristics from your estimated forces
From your **small-slip** masked points:
- estimate front and rear **cornering stiffness** (report the fitted slopes and the slip-angle range used)

From your **high-slip** data:
- estimate the **slip limits** where forces saturate
- estimate the effective **friction coefficient** (state your assumptions from class notes)

### B. Fiala model fit (required)
Fit the Fiala model parameters using the expressions from class notes:
- **Front tire:** pure slip model
- **Rear tire:** combined slip model

Deliverables:
- overlay plots showing:
  - estimated forces (scatter)
  - fitted Fiala model prediction (curve)
- a short discussion of fit quality and any systematic mismatch
- The parameters you identified for the Fiala model (report the values and units)

### C. Pacejka front model (graduate only)
Fit a Pacejka pure-slip model for the **front lateral** force:
- report the identified parameters
- overlay the model on the same front data used for Fiala

---

## What you submit: A single docx + code in appendix

