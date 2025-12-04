import numpy as np
import matplotlib.pyplot as plt

# ------------------------
# Parameters
# ------------------------
N_theta = 32  # total projections
K = 4  # number of interlaced loops
r_outer = 1.0  # radius for first loop
r_step = 0.15  # radial step between loops


# ------------------------
# Universal bit-reversal permutation for ANY K
# ------------------------
def bit_reverse_generalized(K):
    bits = int(np.ceil(np.log2(K)))
    raw_vals = []

    for x in range(K):
        b = f'{x:0{bits}b}'
        rev = int(b[::-1], 2)
        raw_vals.append(rev)

    # rank to ensure a valid permutation 0..K-1
    order = np.argsort(raw_vals)
    ranks = np.zeros(K, dtype=int)
    for r, idx in enumerate(order):
        ranks[idx] = r

    return ranks


# Precompute permutation
perm = bit_reverse_generalized(K)


# ------------------------
# Compute acquisition angles in time order
# ------------------------
angles = []
loop_indices = []

for n in range(N_theta):
    loop = (n * K // N_theta) % K
    rev = perm[loop]                # <-- USE UNIVERSAL BIT-REVERSE
    val = n * K + rev

    theta = val * 2 * np.pi / N_theta
    angles.append(theta)
    loop_indices.append(loop)


angles = np.array(angles)
loop_indices = np.array(loop_indices)

# ------------------------
# Assign radius based on loop
# ------------------------
radii = r_outer - loop_indices * r_step

# ------------------------
# Plot acquisition sequence
# ------------------------
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, polar=True)
ax.set_title(f"TIMBIR Interlaced Acquisition (N={N_theta} - K={K})\nEach loop on its own circle",
             va='bottom', fontsize=13)

ax.plot(angles, radii, '-o', lw=1.2, ms=5, alpha=0.8, color='tab:blue')

for i in range(N_theta):
    ax.text(angles[i], radii[i] + 0.03,
            str(loop_indices[i] + 1), ha='center', va='bottom', fontsize=8)

ax.set_rticks([])
plt.show()

#-------------------------------------------
#         ADD
#-------------------------------------------
angles_deg = np.degrees(angles)
angles_deg_rounded = np.round(angles_deg, 4)
angles_deg_uniq = np.unique(angles_deg_rounded)

angles_str = ', '.join(f'{a:.2f}' for a in angles_deg)
angles_str_uniq = ', '.join(f'{a:.2f}' for a in angles_deg_uniq)

print("Acquisition angles (degrees) in time order:")
print(angles_str)
print("Acquisition angles uniq (degrees):")
print(angles_str_uniq)
