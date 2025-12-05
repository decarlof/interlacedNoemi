import numpy as np
from epics import PV

# ----------------------------------------------------
# Lettura PV
# ----------------------------------------------------
pv_N_theta = PV("2bmb:TomoScan:NTheta")   # NUOVO PV
pv_K = PV("2bmb:TomoScan:KLoops")         # NUOVO PV  




# --------------------------------------------------------------------
# Interlaced GOLDEN
# --------------------------------------------------------------------
def generate_golden_interlaced_angles():
    # ----------------------------------------------------
    # Lettura PV
    # ----------------------------------------------------
    pv_N_theta = PV("2bmb:TomoScan:NTheta")
    pv_K = PV("2bmb:TomoScan:KLoops")

    N_theta = int(pv_N_theta.get() or 32)  # default 32 se PV non risponde
    K = int(pv_K.get() or 4)               # default 4 se PV non risponde

    # --------------------------------------------------------------------
    # Parametri
    # --------------------------------------------------------------------
    end_angle = 360.
    golden_a = end_angle * (3 - np.sqrt(5)) / 2  # ≈ 111.246°
    num_proj = N_theta  # numero di proiezioni per loop

    theta_start = np.linspace(0, end_angle, K, endpoint=False)  # offset interlacciamento equispaziato

    # --------------------------------------------------------------------
    # Genero angoli 
    # --------------------------------------------------------------------
    golden_angles_tomo = np.mod(
        theta_start[:, None] + np.arange(num_proj) * golden_a,
        end_angle
    ).flatten()

    golden_angles_interl = np.unique(np.sort(golden_angles_tomo))  # rimozione duplicati
    return golden_angles_interl


# --------------------------------------------------------------------
# Interlaced TIMBIR
# --------------------------------------------------------------------
def generate_timbir_interlaced_angles():
    pv_N_theta = PV("2bmb:TomoScan:NTheta")   # numero totale proiezioni
    pv_K = PV("2bmb:TomoScan:KLoops")         # numero di loop interlacciati

    N_theta = int(pv_N_theta.get() or 32)  # numero totale proiezioni 
    K = int(pv_K.get() or 4)               # numero di loop interlacciati 

    # ----------------------------------------------------
    # BIT-REVERSAL
    # ----------------------------------------------------
    def bit_reverse(x, bits):
        b = f'{x:0{bits}b}'
        return int(b[::-1], 2)

    # ----------------------------------------------------
    # TIMBIR
    # ----------------------------------------------------
    angles_timbir = []
    loop_indices = []
    bits = int(np.log2(K)) 

    for n in range(N_theta):
        base = n * K
        loop = (base // N_theta) % K
        rev = bit_reverse(loop, bits)
        val = base + rev

        theta = val * 360.0 / N_theta       # angolo 0-360°
        theta = theta % 180.0               # 0-180° per tomografia

        angles_timbir.append(theta)
        loop_indices.append(loop)

    angles_timbir = np.array(angles_timbir)
    loop_indices = np.array(loop_indices)

    print("Angles TIMBIR (degrees):")
    print(np.round(angles_timbir, 4))

    return angles_timbir, np.array(angles_timbir), loop_indices

