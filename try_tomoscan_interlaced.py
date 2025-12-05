import numpy as np
import matplotlib.pyplot as plt

# ------------------------
# Parameters
# ------------------------
N_theta = 32   # total projections
K = 4          # number of interlaced loops
r_outer = 1.0  # radius for first loop
r_step = 0.15  # radial step between loops

# ------------------------
# Bit-reversal function
# ------------------------
def bit_reverse(x, bits):
    b = f'{x:0{bits}b}'
    return int(b[::-1], 2)

# ------------------------
# Compute acquisition angles_timbir  in time order
# ------------------------
bits = int(np.log2(K))
angles_timbir = []
loop_indices = []

for n in range(N_theta):
    val = n * K + bit_reverse((n * K // N_theta) % K, bits)
    theta = val * 2 * np.pi / N_theta  # full 360° rotation
    angles_timbir .append(theta)
    
    # Determine which loop this acquisition belongs to
    loop = (n * K // N_theta) % K   # 0 to K-1
    loop_indices.append(loop)

angles_timbir  = np.array(angles_timbir )
loop_indices = np.array(loop_indices)

print('Angoli interlacciati:'angles_timbir )
return angles_timbir, np.array(angles_timbir), loop_indices

# ------------------------
# Assign radius based on loop
# ------------------------
radii = r_outer - loop_indices * r_step  # all points in the same loop share radius

# ------------------------
# Plot acquisition sequence
# ------------------------
fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(111, polar=True)
ax.set_title(f"TIMBIR Interlaced Acquisition (N={N_theta} - K={K})\nEach loop on its own circle", va='bottom', fontsize=13)

# Connect points in true acquisition order
ax.plot(angles_timbir , radii, '-o', lw=1.2, ms=5, alpha=0.8, color='tab:blue')

# Optional: annotate loop number
for i in range(N_theta):
    ax.text(angles_timbir [i], radii[i]+0.03, str(loop_indices[i]+1), ha='center', va='bottom', fontsize=8)

# Hide radial ticks
ax.set_rticks([])
plt.show()

#-----------------------------------------------------------------
# Simula Counts encoder  e impulsi 
#-----------------------------------------------------------------
PSOCountsPerRotation = 20000           # 1 impulso = 0.018                          # counts_per_rev = PV("2bmb:TomoScan:PSOCountsPerRotation") # Numero di impulsi per giro del PSO

encoder_multiply = PSOCountsPerRotation  / 360.             # fattore di conversione da angoli a impulsi 
raw_delta_encoder_counts =  angles_timbir * encoder_multiply 
delta_encoder_counts = np.round(raw_delta_encoder_counts).astype(int)


rotation_step_real_deg = delta_encoder_counts / encoder_multiply   # Rotation step reale (dopo arrotondamento)


plt.figure(figsize=(12,5))

plt.subplot(1,2,1)
plt.plot(angles_timbir, delta_encoder_counts, '.-')
plt.xlabel("Angolo ideale (°)")
plt.ylabel("Count PSO (arrotondato)")
plt.title("Angolo in impulsi PSO (con arrotondamento)")
plt.grid(True)

plt.subplot(1,2,2)
plt.plot(angles_timbir, rotation_step_real_deg, '.-')
plt.xlabel("Angolo ideale (°)")
plt.ylabel("Angolo reale (°)")
plt.title("Impulsi PSO → Angolo reale (dopo arrotondamento)")
plt.grid(True)

plt.tight_layout()
plt.show()




















