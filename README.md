# Balance Control of a Segway Robot Using Classical and Modern Control Techniques

This repository contains the MATLAB/Simulink files developed for a research project on balancing a two-wheeled self-balancing (Segway-type) robot. The robot is modeled as an inverted-pendulum-on-a-cart system and is stabilized in the upright position using:

- **Classical control** — PID controllers (manually tuned and MATLAB `pidtune`-assisted)
- **Modern control** — Linear Quadratic Regulator (LQR) state feedback
- **Optimization** — Multi-objective Genetic Algorithm (GA) tuning of the LQR weighting matrices (Q, R)

Two independent modeling/actuation approaches are implemented and kept in separate folders:

- **By Force Approach** — the classic cart-pole (cart-and-inverted-pendulum) model, actuated by a linear force applied to the cart.
- **By Torque Approach** — a wheeled-chassis model of the Segway, actuated by a torque applied at the wheels.

## Repository Structure

```
.
├── By Force Approach/
│   ├── Calculating_LQR_Gain_By_Force.m                        # State-space model + manually chosen Q, R -> LQR gain K
│   ├── Segway_Robot_Using_LQR_Controller_By_Force.slx         # Simulink model: cart-pole plant + LQR state-feedback controller
│   ├── Segway_Robot_Using_PID_Controller_Block_By_Force.slx   # Simulink model: cart-pole plant + PID Controller block
│   ├── Segway_Robot_Using_LQR_Controller_By_Force.mp4         # Recorded simulation (LQR, manually chosen Q/R)
│   ├── Segway_Robot_Using_LQR_Controller_By_Force.gif         # GIF preview of the above, for inline README embedding
│   └── Genetic Algorithm for LQR/
│       ├── Calculating_LQR_Gain_By_Force.m                    # Same plant model, loaded with the GA-optimized Q, R
│       ├── GA_Optimization_for_LQR.m                          # Multi-objective GA (gamultiobj) that searches Q, R for the LQR gain
│       ├── multiObjectiveFunction.m                           # Objective function: runs the Simulink model and returns ITAE cost for position & angle
│       ├── Segway_Robot_Using_LQR_Controller_By_Force.slx     # Simulink model used by the GA as its plant/controller under test
│       ├── Segway_Robot_Using_LQR_Controller_By_Force.mp4     # Recorded simulation (LQR, GA-optimized Q/R)
│       ├── Segway_Robot_Using_LQR_Controller_By_Force_GA.gif  # GIF preview of the above, for inline README embedding
│       └── GA_Results/
│           ├── GA_Pareto_Graph.png                            # Pareto front (Cost1 = ITAE position vs Cost2 = ITAE angle)
│           └── GA_Final_Results.png                           # Table of Pareto-optimal (q1..q4, r, K, Cost1, Cost2) solutions
├── By Torque Approach/
│   ├── Tuning_PID_Gains.m                                     # Transfer-function model (torque input) + pidtune -> PID gains
│   ├── Self_Balancing_Robot_Using_PID_Controller_Manual_Tuning.slx  # Simulink model: wheeled-chassis plant + PID controller
│   ├── Segway_Robot_Using_PID_Controller_By_Torque.mp4        # Recorded simulation (PID, torque actuation)
│   └── Segway_Robot_Using_PID_Controller_By_Torque.gif        # GIF preview of the above, for inline README embedding
├── Complete_Project_Report.pdf                                 # Full project report (theory, derivations, results)
└── Final_Project_Presentation.pptx                             # Final project presentation slides
```

## Prerequisites

To open the models and re-run the simulations/optimizations yourself, you need:

- **MATLAB** (R2021b or later recommended)
- **Simulink**
- **Control System Toolbox** — required for `lqr`, `tf`, `pidtune`, state-space analysis
- **Global Optimization Toolbox** — required only for the GA script (`gamultiobj`) in `By Force Approach/Genetic Algorithm for LQR/`

No other third-party libraries or add-ons are required. All models were built with standard Simulink blocks (State-Space, PID Controller, Scope, etc.).

## Getting Started

1. **Clone the repository**

   ```bash
   git clone https://github.com/Preetamk97/Balance-Control-of-Segway-Robot-Using-Classical-And-Mordern-Control-Techniques.git
   cd Balance-Control-of-Segway-Robot-Using-Classical-And-Mordern-Control-Techniques
   ```

2. **Open MATLAB** and set the current folder to the cloned repository (or the specific sub-folder you want to work with), so that any `.m` script can find its companion `.slx` model on the path.

3. Pick one of the workflows below depending on which controller/approach you want to reproduce.

---

## Workflow 1 — "By Force Approach": Cart-Pole Model

The robot is modeled as a cart (chassis) with an inverted pendulum (the body) mounted on it, actuated by a horizontal force. State vector: cart position, cart velocity, pendulum angle, angular velocity.

### A. LQR Controller (manually tuned Q, R)

1. Open `By Force Approach/Calculating_LQR_Gain_By_Force.m` in MATLAB.
2. Run the script. It builds the state-space matrices `A, B, C, D` from the physical parameters (cart mass, pendulum mass, friction, pendulum length, etc.), defines the LQR weighting matrices `Q` and `R`, and computes the gain `K = lqr(A, B, Q, R)`. `K` is left in the MATLAB workspace.
3. Open `By Force Approach/Segway_Robot_Using_LQR_Controller_By_Force.slx` in Simulink. The model reads `K` from the base workspace and uses it in a state-feedback block.
4. Click **Run** in Simulink to simulate. Use the model's Scope blocks to view the cart position and pendulum angle responses — this reproduces `Segway_Robot_Using_LQR_Controller_By_Force.mp4`.

### B. PID Controller

1. Open `By Force Approach/Segway_Robot_Using_PID_Controller_Block_By_Force.slx` directly in Simulink (it uses a standard PID Controller block, no separate MATLAB script is required to set gains — the block already contains the tuned values).
2. Click **Run** and observe the Scope outputs.

### C. GA-Optimized LQR Gains

The `Genetic Algorithm for LQR` sub-folder searches for the best LQR weighting matrices `Q = diag(q1,q2,q3,q4)` and `R` using a multi-objective Genetic Algorithm that minimizes two ITAE (Integral of Time-weighted Absolute Error) costs simultaneously: cart-position error and pendulum-angle error.

1. Make sure `By Force Approach/Genetic Algorithm for LQR/` is your MATLAB current folder (the plant model and the Simulink `.slx` file must be visible on the path).
2. Run `GA_Optimization_for_LQR.m`. This script:
   - Builds the state-space matrices `A, B, C, D` and pushes them to the base workspace.
   - Runs `gamultiobj` over decision variables `[q1 q2 q3 q4 r]`, with each candidate evaluated by `multiObjectiveFunction.m`, which computes `K = lqr(A,B,Q,R)`, runs `Segway_Robot_Using_LQR_Controller_By_Force.slx` in that folder via `sim(...)`, and reads back the ITAE-position and ITAE-angle scopes/signals (`ITAE1`, `ITAE2`) as the two objective costs.
   - Displays a live Pareto plot during optimization and prints a results table of all Pareto-optimal `(q1, q2, q3, q4, r, K, Cost1, Cost2)` solutions at the end (see `GA_Results/GA_Pareto_Graph.png` and `GA_Results/GA_Final_Results.png` for a sample run's output).
3. Pick a preferred trade-off point from the Pareto front, copy its `q1..q4, r` values into `Calculating_LQR_Gain_By_Force.m` (or directly set `Q`/`R` and rerun `lqr`) to get the final gain `K`.
4. With `K` in the workspace, open and run `Segway_Robot_Using_LQR_Controller_By_Force.slx` in this folder to visualize the response — this reproduces the GA-tuned `Segway_Robot_Using_LQR_Controller_By_Force.mp4` video in this sub-folder.

> Note: `gamultiobj` runs the full Simulink simulation once per candidate solution per generation, so this optimization can take a while (population size and generation count are set inside `GA_Optimization_for_LQR.m` — reduce `PopulationSize`/`MaxGenerations` for a quicker, less-refined search).

---

## Workflow 2 — "By Torque Approach": Wheeled-Chassis Model, PID Control

Here the Segway is modeled with wheel and chassis parameters (chassis mass, wheel mass/radius, moments of inertia) and actuated by a torque at the wheels. The plant is derived as a transfer function relating the chassis tilt angle to the input torque.

1. Open `By Torque Approach/Tuning_PID_Gains.m` in MATLAB and run it.
   - It builds the physical constants and combined coefficients (`k1..k5`), forms the plant transfer function `plantTF` (tilt angle vs. torque input), and calls `pidtune(plantTF, 'PID')` to automatically compute `Kp`, `Ki`, `Kd`.
   - The tuned gains are printed to the command window.
2. Open `By Torque Approach/Self_Balancing_Robot_Using_PID_Controller_Manual_Tuning.slx` in Simulink and enter/verify the `Kp`, `Ki`, `Kd` values in the model's PID Controller block (either the values from step 1, or the manually tuned values already saved in the model).
3. Click **Run** to simulate — this reproduces `Segway_Robot_Using_PID_Controller_By_Torque.mp4`.

---

## Reference Material

- **`Complete_Project_Report.pdf`** — full write-up of the mathematical modeling (both cart-pole and wheeled-chassis derivations), controller design, GA-based optimization methodology, and simulation results.
- **`Final_Project_Presentation.pptx`** — condensed slide-deck summary of the project.

## Simulation Results (Animated Previews)

GitHub does not render `<video>` tags or play `.mp4` files inline inside a README, so the recordings below are embedded as GIF previews (converted from the original `.mp4` files, which are also kept in this repository at full quality — see the links under each preview).

### By Force Approach — LQR Controller (Manually Tuned Q, R)

![LQR Controller Simulation - Manually Tuned Q,R](By%20Force%20Approach/Segway_Robot_Using_LQR_Controller_By_Force.gif)

Full-quality video: [`Segway_Robot_Using_LQR_Controller_By_Force.mp4`](By%20Force%20Approach/Segway_Robot_Using_LQR_Controller_By_Force.mp4)

![LQR Manual Position v/s Time Graph](By%20Force%20Approach/Results/LQR-Manual_Postion-Time-Graph.png) <br>
![LQR Manual Chasis Tilt Angle v/s Time Graph](By%20Force%20Approach/Results/LQR-Manual-TiltAngle-Time_Graph.png)

### By Force Approach — LQR Controller (GA-Optimized Q, R)

![LQR Controller Simulation - GA Optimized Q,R](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/Segway_Robot_Using_LQR_Controller_By_Force_GA.gif)

Full-quality video: [`Segway_Robot_Using_LQR_Controller_By_Force.mp4`](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/Segway_Robot_Using_LQR_Controller_By_Force.mp4)

![GA Optimised LQR Manual Position v/s Time Graph](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/GA_Results/Position-Time-Graph.png)

![GA Optimised LQR Manual Chasis Tilt Angle v/s Time Graph](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/GA_Results/Chassis-TiltAngle-Time-Graph.png)

### GA Optimization Results

**Pareto front** (Cost 1 = ITAE of cart position, Cost 2 = ITAE of pendulum angle):

![GA Pareto Front](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/GA_Results/GA_Pareto_Graph.png)

**Pareto-optimal solutions table** (`q1, q2, q3, q4, r, K, Cost1, Cost2`):

![GA Final Results](By%20Force%20Approach/Genetic%20Algorithm%20for%20LQR/GA_Results/GA_Final_Results_Marked.png)

### By Torque Approach — PID Controller

![PID Controller Simulation - Torque Approach](By%20Torque%20Approach/Segway_Robot_Using_PID_Controller_By_Torque.gif)

Full-quality video: [`Segway_Robot_Using_PID_Controller_By_Torque.mp4`](By%20Torque%20Approach/Segway_Robot_Using_PID_Controller_By_Torque.mp4)

![PID Controller Position v/s Time Graph](By%20Torque%20Approach/Results/PID-Graph.png)

## Author

Pritam Ranjan Kalita ([preetamk97@gmail.com](mailto:preetamk97@gmail.com))
