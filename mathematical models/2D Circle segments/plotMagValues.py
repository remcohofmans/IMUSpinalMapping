import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV file
CSV_FILE = "magnetometer_data.csv"
df = pd.read_csv(CSV_FILE)

# Extract columns (Make sure CSV has X, Y, Z)
raw_x = df["X"]
raw_y = df["Y"]
raw_z = df["Z"]

# Create figure
fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)  # Keep aspect ratio 1:1

# Scatter plots for each axis combination
sc1 = ax.scatter(raw_x, raw_y, color="r", label="X vs Y")
sc2 = ax.scatter(raw_y, raw_z, color="g", label="Y vs Z")
sc3 = ax.scatter(raw_z, raw_x, color="b", label="Z vs X")

# Labels and legend
ax.set_xlabel("Magnetometer Values")
ax.set_ylabel("Magnetometer Values")
ax.legend()
plt.title("2D Scatter Plots of Raw Magnetometer Data")


max_x = max(raw_x)
max_y = max(raw_y)
max_z = max(raw_z)

min_x = min(raw_x)
min_y = min(raw_y)
min_z = min(raw_z)

print(f"Max X: {max_x}, Min X: {min_x}")
print(f"Max Y: {max_y}, Min Y: {min_y}")
print(f"Max Z: {max_z}, Min Z: {min_z}")

offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2
print(f"Offset X: {offset_x}, Offset Y: {offset_y}, Offset Z: {offset_z}")

# Apply offset to raw data
centered_x = raw_x - offset_x
centered_y = raw_y - offset_y
centered_z = raw_z - offset_z

# Create figure
fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)  # Keep aspect ratio 1:1

# Scatter plots for each axis combination
sc1 = ax.scatter(centered_x, centered_y, color="r", label="X vs Y")
sc2 = ax.scatter(centered_y, centered_z, color="g", label="Y vs Z")
sc3 = ax.scatter(centered_z, centered_x, color="b", label="Z vs X")

# Labels and legend
ax.set_xlabel("Magnetometer Values")
ax.set_ylabel("Magnetometer Values")
ax.legend()
plt.title("2D Scatter Plots of biased Magnetometer Data")

#calculate covariance matrix - like below but in python
cov = np.zeros((3, 3))

# Calculate covariance matrix
for i in range(len(centered_x)):
    cov[0][0] += centered_x[i] * centered_x[i]
    cov[0][1] += centered_x[i] * centered_y[i]
    cov[0][2] += centered_x[i] * centered_z[i]
    cov[1][1] += centered_y[i] * centered_y[i]
    cov[1][2] += centered_y[i] * centered_z[i]
    cov[2][2] += centered_z[i] * centered_z[i]
    
cov[1][0] = cov[0][1]
cov[2][0] = cov[0][2]
cov[2][1] = cov[1][2]

def jacobi_eigen_decomposition(cov, max_iter=50, epsilon=1e-10):
    """Performs eigenvalue decomposition using the Jacobi method."""
    n = cov.shape[0]
    eigenvectors = np.eye(n)
    a = cov.copy()
    
    for _ in range(max_iter):
        # Find largest off-diagonal element
        max_val = 0.0
        p, q = 0, 1
        for i in range(n):
            for j in range(i + 1, n):
                if abs(a[i, j]) > max_val:
                    max_val = abs(a[i, j])
                    p, q = i, j
        
        # Check for convergence
        if max_val < epsilon:
            break
        
        # Compute rotation parameters
        theta = 0.5 * np.arctan2(2.0 * a[p, q], a[p, p] - a[q, q])
        c, s = np.cos(theta), np.sin(theta)
        
        # Apply rotation
        for i in range(n):
            if i != p and i != q:
                a_ip, a_iq = a[i, p], a[i, q]
                a[i, p] = c * a_ip + s * a_iq
                a[i, q] = -s * a_ip + c * a_iq
                a[p, i] = a[i, p]
                a[q, i] = a[i, q]
        
        # Update diagonal elements
        a_pp, a_qq, a_pq = a[p, p], a[q, q], a[p, q]
        a[p, p] = c**2 * a_pp + s**2 * a_qq + 2 * c * s * a_pq
        a[q, q] = s**2 * a_pp + c**2 * a_qq - 2 * c * s * a_pq
        a[p, q] = a[q, p] = 0  # Zero out the rotated element
        
        # Update eigenvectors
        for i in range(n):
            v_ip, v_iq = eigenvectors[i, p], eigenvectors[i, q]
            eigenvectors[i, p] = c * v_ip + s * v_iq
            eigenvectors[i, q] = -s * v_ip + c * v_iq
    
    return np.diag(a), eigenvectors

def compute_soft_iron_matrix(cov, sample_count):
    """Computes the soft iron correction matrix."""
    # Normalize covariance matrix
    cov /= sample_count
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    # Ensure positive eigenvalues (numerical stability)
    eigenvalues = np.maximum(eigenvalues, 1e-10)
    
    # Compute scaling factors
    avg_radius = np.sqrt(np.mean(eigenvalues))
    D = np.diag(avg_radius / np.sqrt(eigenvalues))
    
    # Compute soft iron correction matrix: V * D * V^T
    VD = eigenvectors @ D
    soft_iron_matrix = VD @ eigenvectors.T
    
    return soft_iron_matrix

sample_count = 5000
soft_iron_matrix = compute_soft_iron_matrix(cov, sample_count)
print("Soft Iron Correction Matrix:")
print(soft_iron_matrix)

calibrated_x = np.zeros(len(centered_x))
calibrated_y = np.zeros(len(centered_y))
calibrated_z = np.zeros(len(centered_z))

for i in range(len(centered_x)):
    calibrated_x[i] = soft_iron_matrix[0][0] * centered_x[i] + soft_iron_matrix[0][1] * centered_y[i] + soft_iron_matrix[0][2] * centered_z[i]
    calibrated_y[i] = soft_iron_matrix[1][0] * centered_x[i] + soft_iron_matrix[1][1] * centered_y[i] + soft_iron_matrix[1][2] * centered_z[i]
    calibrated_z[i] = soft_iron_matrix[2][0] * centered_x[i] + soft_iron_matrix[2][1] * centered_y[i] + soft_iron_matrix[2][2] * centered_z[i]

# Create figure
fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)  # Keep aspect ratio 1:1

# Scatter plots for each axis combination
sc2 = ax.scatter(calibrated_y, calibrated_z, color="g", label="Y vs Z")
sc3 = ax.scatter(calibrated_z, calibrated_x, color="b", label="Z vs X")
sc1 = ax.scatter(calibrated_x, calibrated_y, color="r", label="X vs Y")


# Labels and legend
ax.set_xlabel("Magnetometer Values")
ax.set_ylabel("Magnetometer Values")
ax.legend()
plt.title("2D Scatter Plots of calculated Magnetometer Data")

# Show plot
plt.show()
