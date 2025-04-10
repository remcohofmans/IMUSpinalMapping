import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV file
CSV_FILE = "LSM303_Mag.csv"
df = pd.read_csv(CSV_FILE)

# Extract columns (Make sure CSV has X, Y, Z)
raw_x = df["X"]
raw_y = df["Y"]
raw_z = df["Z"]

# Calculate mean values as offsets
offset_x = (max(raw_x) + min(raw_x))/2
offset_y = (max(raw_y) + min(raw_y))/2    
offset_z = (max(raw_z) + min(raw_z))/2
print(f"Offset X: {offset_x}, Offset Y: {offset_y}, Offset Z: {offset_z}")

# Apply offset to raw data
centered_x = raw_x - offset_x
centered_y = raw_y - offset_y
centered_z = raw_z - offset_z

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
plt.title("2D Scatter Plots of biased Magnetometer Data")

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

# Calculate covariance matrix
cov = np.zeros((3, 3))
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

def compute_soft_iron_matrix(cov, sample_count):
    """Computes the soft iron correction matrix."""
    cov /= sample_count
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    eigenvalues = np.maximum(eigenvalues, 1e-10)
    avg_radius = np.sqrt(np.mean(eigenvalues))
    D = np.diag(avg_radius / np.sqrt(eigenvalues))
    VD = eigenvectors @ D
    soft_iron_matrix = VD @ eigenvectors.T
    return soft_iron_matrix

sample_count = len(centered_x)
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
sc1 = ax.scatter(calibrated_x, calibrated_y, color="r", label="X vs Y")
sc2 = ax.scatter(calibrated_y, calibrated_z, color="g", label="Y vs Z")
sc3 = ax.scatter(calibrated_z, calibrated_x, color="b", label="Z vs X")


# Labels and legend
ax.set_xlabel("Magnetometer Values")
ax.set_ylabel("Magnetometer Values")
ax.legend()
plt.title("2D Scatter Plots of calculated Magnetometer Data")

# Show plot
plt.show()
