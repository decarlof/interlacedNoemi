import numpy as np
import matplotlib.pyplot as plt

#-----------------------------------------------------------------
# Parametri TIMBIR
#-----------------------------------------------------------------
N_theta = 32          # total projections
K = 4                 # number of interlaced loops
r_outer = 1.0         # radius for first loop (plot only)
r_step = 0.15         # radial step between loops

#-----------------------------------------------------------------
# Funzione di bit-reversal
#-----------------------------------------------------------------
def bit_reverse(x, bits):
    b = f'{x:0{bits}b}'
    return int(b[::-1], 2)

#-----------------------------------------------------------------
# TIMBIR angle generator (IN ORDER OF ACQUISITION)
#-----------------------------------------------------------------
def generate_timbir_angles(N_theta, K):
    bits = int(np.log2(K))
    angles = []
    loops = []

    for n in range(N_theta):
        # TIMBIR index logic
        group = (n * K // N_theta) % K
        idx = n * K + bit_reverse(group, bits)

        # convert index -> degrees (0-360)
        theta_deg = (idx % N_theta) * 360.0 / N_theta
        angles.append(theta_deg)
        loops.append(group)

    return np.array(angles), np.array(loops)

# compute TIMBIR angles
angles_timbir, loop_indices = generate_timbir_angles(N_theta, K)

#-----------------------------------------------------------------
# Convert ANGLES → ABSOLUTE PULSES
#-----------------------------------------------------------------
absolute_pulses = np.round(angles_timbir * pulses_per_degree).astype(int)

#-----------------------------------------------------------------
# Convert ABSOLUTE PULSES → DELAYS (this is what FPGA needs)
#-----------------------------------------------------------------
# FPGA wants: N[i] = pulses[i+1] - pulses[i]
delays = np.diff(absolute_pulses)

# ensure no zeros or negative delays (FPGA cannot handle them)
# if angles wrap around or unordered, sort OR add 20000 to unwrap
delays[delays <= 0] += PSOCountsPerRotation

#-----------------------------------------------------------------
# DEBUG PLOT: angle → pulses → delays
#-----------------------------------------------------------------
plt.figure(figsize=(12,5))

plt.subplot(1,2,1)
plt.plot(angles_timbir, absolute_pulses, "o-", label="pulses")
plt.xlabel("Angles (deg)")
plt.ylabel("Absolute pulses")
plt.title("Angles → encoder pulses")
plt.grid()

plt.subplot(1,2,2)
plt.plot(delays, "o-", label="delays (FPGA input)")
plt.xlabel("Projection index")
plt.ylabel("Δ pulses (divide-by-N)")
plt.title("Pulse delays for FPGA")
plt.grid()

plt.tight_layout()
plt.show()

#-----------------------------------------------------------------
# PRINT OUTPUT FOR FPGA
#-----------------------------------------------------------------
print("\n=== ANGOLI TIMBIR (deg) ===")
print(angles_timbir)

print("\n=== IMPULSI ASSOLUTI ===")
print(absolute_pulses)

print("\n=== DELAYS (FPGA READY) ===")
print(delays)
import numpy as np

def simulate_taxi_motion(accel, decel, omega_target, theta_total, dt=1e-4):
    """
    Simula un fly-scan realistico:
      - accel. con accelerazione costante
      - regime a omega_target
      - decelerazione con decel costante
    
    Restituisce:
        t_vec        → ascissa temporale
        theta_real   → angolo reale percorso
        omega_real   → velocità istantanea
    """
    # ------------------------------
    # 1. Accelerazione
    # ------------------------------
    T_acc = omega_target / accel
    t_acc = np.arange(0, T_acc, dt)
    theta_acc = 0.5 * accel * t_acc**2

    # ------------------------------
    # 2. Plateau
    # ------------------------------
    theta_acc_end = theta_acc[-1]
    theta_to_cover = theta_total - 2 * theta_acc_end

    T_flat = theta_to_cover / omega_target
    t_flat = np.arange(0, T_flat, dt)
    theta_flat = theta_acc_end + omega_target * t_flat

    # ------------------------------
    # 3. Decelerazione
    # ------------------------------
    T_dec = omega_target / decel
    t_dec = np.arange(0, T_dec, dt)
    theta_dec = theta_flat[-1] + omega_target*t_dec - 0.5*decel*t_dec**2

    # ------------------------------
    # Concatenazione
    # ------------------------------
    t_vec = np.concatenate([
        t_acc,
        t_acc[-1] + t_flat,
        t_acc[-1] + t_flat[-1] + t_dec
    ])

    theta_real = np.concatenate([theta_acc, theta_flat, theta_dec])
    omega_real = np.gradient(theta_real, dt)

    return t_vec, theta_real, omega_real


def invert_theta(theta_real, t_vec, theta_targets):
    """
    Interpola i tempi reali in cui lo stage raggiunge gli angoli target.
    """
    return np.interp(theta_targets, theta_real, t_vec)


def angles_to_real_pulses(theta_real, pulses_per_degree):
    return np.round(theta_real * pulses_per_degree).astype(int)


def pulses_to_delays(pulses_real, pulses_per_rev):
    delays = np.diff(pulses_real)

    # FPGA: N deve essere sempre > 0
    delays[delays <= 0] += pulses_per_rev

    return delays


def compute_taxi_corrected_delays(angles_ideal,
                                  pulses_per_degree,
                                  pulses_per_rev,
                                  accel,
                                  decel,
                                  omega_target,
                                  theta_total):
    """
    Pipeline completa:
       1. Simula θ_real(t)
       2. Trova t(θ_ideal)
       3. Ricava θ_real(t_real)
       4. Converti in pulses_real
       5. Calcola delays FPGA-ready

 pulses_real → il vero impulso in cui lo stage raggiunge ogni angolo TIMBIR

delays → array esatto per la FPGA, accurato al microsecondo

theta_reached → angolo realmente raggiunto

t_real → tempo esatto del trigger
    """

    # 1) Simulazione profilo reale
    t_vec, theta_real_vec, omega_vec = simulate_taxi_motion(
        accel, decel, omega_target, theta_total
    )

    # 2) Tempi reali dei singoli angoli
    t_real = invert_theta(theta_real_vec, t_vec, angles_ideal)

    # 3) Angolo reale al tempo corretto
    theta_reached = np.interp(t_real, t_vec, theta_real_vec)

    # 4) Converti in impulsi
    pulses_real = angles_to_real_pulses(theta_reached, pulses_per_degree)

    # 5) Converti impulsi → delays FPGA
    delays = pulses_to_delays(pulses_real, pulses_per_rev)

    return pulses_real, delays, theta_reached, t_real



