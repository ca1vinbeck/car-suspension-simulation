import numpy as np
import matplotlib.pyplot as plt

# System parameters
m = 250
dt = 0.01
T = 10
t = np.arange(0, T, dt)

# Initial conditions
xi = 0
vi = 0

# Road profile
def road(t):
    if t < 1:
        return 0
    elif t < 2:
        return 0.05 * (1 - np.cos(np.pi * (t - 1))) / 2
    else:
        return 0.05

# Numerical derivative of road (Second Order Centered Difference)
def road_derivative(t, dt=1e-4):
    return (road(t + dt) - road(t - dt)) / (2 * dt)

# System equations:
# x1 = x, x2 = x'
# x1' = x2
# x2' = (-k*(x1 - y) - c*(x2 - y_dot)) / m (derived from Newton's second law)

# System dynamics
def derivatives(t, IC, k, c):
    x1 = IC[0]
    x2 = IC[1]

    y = road(t)
    y_dot = road_derivative(t)

    dx1dt = x2
    dx2dt = (-k * (x1 - y) - c * (x2 - y_dot)) / m

    return np.array([dx1dt, dx2dt])

# RK4 (DE Solver)
def RK4(f, t, IC, dt, k, c):
    k1 = f(t, IC, k, c)
    k2 = f(t + dt/2, IC + dt/2 * k1, k, c)
    k3 = f(t + dt/2, IC + dt/2 * k2, k, c)
    k4 = f(t + dt, IC + dt * k3, k, c)

    return IC + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

# Simulation function
def simulate(k, c):
    IC = np.array([xi, vi])

    x_vals = np.zeros(t.size)
    v_vals = np.zeros(t.size)
    a_vals = np.zeros(t.size)
    y_vals = np.zeros(t.size)

    for i in range(t.size):
        y = road(t[i])
        y_dot = road_derivative(t[i])

        x_vals[i] = IC[0]
        v_vals[i] = IC[1]
        y_vals[i] = y

        # acceleration from dynamics
        a_vals[i] = (-k * (IC[0] - y) - c * (IC[1] - y_dot)) / m

        IC = RK4(derivatives, t[i], IC, dt, k, c)

    return x_vals, v_vals, a_vals, y_vals

# Parameter Arrays
k_vals = np.linspace(5000, 30000, 20)
c_vals = np.linspace(500, 3000, 20)

# Metrics
A = np.zeros((len(k_vals), len(c_vals)))  # acceleration (comfort)
D = np.zeros((len(k_vals), len(c_vals)))  # displacement (handling)
S = np.zeros((len(k_vals), len(c_vals)))  # settling (stability)

# Evaluate all combinations
for i, k in enumerate(k_vals):
    for j, c in enumerate(c_vals):
        x_vals, v_vals, a_vals, y_vals = simulate(k, c)

        A[i, j] = np.max(np.abs(a_vals))
        D[i, j] = np.max(np.abs(x_vals - y_vals))
        S[i, j] = np.sum(np.abs(v_vals)) * dt

# Flatten for plotting
A_flat = A.flatten()
D_flat = D.flatten()
S_flat = S.flatten()

# Tradeoff plot: Comfort vs Handling
fig, ax = plt.subplots()
ax.scatter(D_flat, A_flat, alpha=0.7, label='All Designs')

ax.set_xlabel("Max Displacement (Handling)")
ax.set_ylabel("Max Acceleration (Comfort)")
fig.suptitle("Suspension Design Tradeoff")

# Highlight extreme designs
# Best comfort occurs at minimum acceleration
min_accel_idx = np.unravel_index(np.argmin(A), A.shape)
# Best handeling occurs at minimum displacement
min_disp_idx = np.unravel_index(np.argmin(D), D.shape)

# Find Pareto Frontier
pareto_mask = np.ones(A_flat.size, dtype=bool)
for i in range(A_flat.size):
    for j in range(D_flat.size):
        if (A_flat[j] <= A_flat[i] and D_flat[j] <= D_flat[i]
            and (A_flat[j] < A_flat[i] or D_flat[j] < D_flat[i])):
            pareto_mask[i] = False
            break


ax.scatter(D_flat[pareto_mask], A_flat[pareto_mask], label='Pareto Frontier')
ax.scatter(D[min_accel_idx], A[min_accel_idx], color='red', label='Min Acceleration (Comfort)', s=10)
ax.scatter(D[min_disp_idx], A[min_disp_idx], color='green', label='Min Displacement (Handling)', s=10)

ax.legend()
plt.show()

# Pick the indices
pareto_indices = np.where(pareto_mask)[0]
middle_idx = pareto_indices[pareto_indices.size//2]  # middle Pareto point

# Convert extreme points to flat indices
min_accel_idx_flat = np.ravel_multi_index(min_accel_idx, A.shape)
min_disp_idx_flat  = np.ravel_multi_index(min_disp_idx, A.shape)

# Pick middle Pareto point
pareto_indices = np.where(pareto_mask)[0]
middle_idx_flat = pareto_indices[len(pareto_indices)//2]

# Collect the systems we want to plot
system_indices = [min_accel_idx_flat, min_disp_idx_flat, middle_idx_flat]
system_labels = ['Best Comfort', 'Best Handling', 'Balanced Pareto']

# Create 3 stacked subplots (shared x-axis)
fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Loop through each system
for i in range(len(system_indices)):
    idx = system_indices[i]  # flat index in A_flat/D_flat
    label = system_labels[i]
    
    # Get the corresponding k and c values from index
    k = k_vals[idx // len(c_vals)]
    c = c_vals[idx % len(c_vals)]
    
    # Run simulation for this system
    x_vals, v_vals, a_vals, y_vals = simulate(k, c)
    
    # Plot displacement (x) and road profile (y)
    axes[0].plot(t, x_vals, label=f'{label} (x)')
    axes[0].plot(t, y_vals, '--', label=f'{label} (road y)')
    
    # Plot velocity
    axes[1].plot(t, v_vals, label=label)
    
    # Plot acceleration
    axes[2].plot(t, a_vals, label=label)

# Add labels and titles
axes[0].set_ylabel('Displacement (m)')
axes[0].set_title('Displacement vs Time')
axes[1].set_ylabel('Velocity (m/s)')
axes[1].set_title('Velocity vs Time')
axes[2].set_ylabel('Acceleration (m/s²)')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('Acceleration vs Time')

# Add legends to first and last subplot (middle one shares)
axes[0].legend()
axes[2].legend()

# Adjust layout so nothing overlaps
plt.tight_layout()
plt.show()
