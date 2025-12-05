import numpy as np
from epics import PV
# ----------------------------------------------------
# Lettura PV e Nuovi PV da aggiungere
# ----------------------------------------------------
pv_N_theta = PV("2bmb:TomoScan:NTheta")   # NUOVO PV
pv_K = PV("2bmb:TomoScan:KLoops")         # NUOVO PV  


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

