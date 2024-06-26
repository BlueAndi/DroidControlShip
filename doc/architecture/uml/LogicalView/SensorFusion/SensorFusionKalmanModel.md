# Kinematic Model of the Extended Kalman Filter
## State Model of the Extended Kalman Filter

The State Vector of the Extended Kalman Filter consists of the following States in the Time Step $k$:
| Symbol | Explanation | Unit |
| - | - | - |
| $p_{\mathrm{x}, k}$ | Position of the Robot in x- direction | $\mathrm{mm}$ |
| $p_{\mathrm{y}, k}$ | Position of the Robot in y- direction | $\mathrm{mm}$ |
| $v_k$ | Tangential velocity | $\mathrm{mm/s}$ |
| $\theta_k$ | Orientation in regard to initial orientation | $\mathrm{mrad}$ |


## System Model
The control input - used in the prediction step of the Extended Kalman Filter - uses the following measurement values:
| Symbol | Explanation |  Unit |
| - | - | - |
| $a_{\mathrm{u}, k}$ | Tangential acceleration, measured by accelerometer | $\mathrm{mm/s^2}$ |
| $\dot{\theta}_k$ | Turn rate, measured by gyroscope | $\mathrm{mrad/s}$ |
| $\Delta t$ | Time since last time step | $\mathrm{ms}$ |


The formula of the system model of the Extended Kalman Filter is defined as:
```math
\begin{bmatrix}
        p_{\mathrm{x}} \\
        p_{\mathrm{y}} \\
        v              \\
        \theta  
    \end{bmatrix}_k
    =
    \begin{bmatrix}
        p_{\mathrm{x}} \\
        p_{\mathrm{y}} \\
        v              \\
        \theta
    \end{bmatrix}_{k-1}
    +
    \begin{bmatrix}
        \triangle t \, v_{k-1} \cos{\theta_{k-1}}  \\
        \triangle t \, v_{k-1} \sin{\theta_{k-1} } \\
        \triangle t \, a_{\mathrm{u},{k-1}}        \\
        \triangle t \, \dot{\theta}_{k-1}
    \end{bmatrix}$$ 
```


## Measurement Model
The measurement Model - used in the update step of the Extended Kalman Filter - uses the following information from the Odometry Unit:
| Symbol | Explanation | Unit |
| - | - | - |
| $\Delta d_{\mathrm{odo}, k}$ | Travelled euclidean distance since last Time Step estimated by Odometry | $\mathrm{mm}$ |
| $\Delta \theta_{\mathrm{odo}, k}$ | Orientation change since last Time Step estimated by Odometry | $\mathrm{mrad}$ |
| $\gamma$ | Coefficient if Robot drives forwards ($\gamma$=1) or backwards ($\gamma$=-1) | - |


The formula of the measurement model of the Extended Kalman Filter is defined as:
```math
\begin{bmatrix}
        p_{\mathrm{x}} \\
        p_{\mathrm{y}} \\
        v              \\
        \theta  
    \end{bmatrix}_k
    =
    \begin{bmatrix}
        p_{\mathrm{x},k-1} + \gamma \, \Delta d_{\mathrm{odo}} \cos{\theta_{k-1}}\\
        p_{\mathrm{y},k-1} + \gamma \, \Delta d_{\mathrm{odo}} \sin{\theta_{k-1}}\\
        \gamma \frac{\Delta d_{\mathrm{odo}, k}}{\Delta t}  \\
        \theta_{k-1} + \Delta \theta_{\mathrm{odo}, k}
    \end{bmatrix}

```

