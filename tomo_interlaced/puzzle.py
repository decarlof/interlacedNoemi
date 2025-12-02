import numpy as np
from epics import PV

# ----------------------------------------------------
# TIMBIR e nuovi PVs 
# ----------------------------------------------------

pv_N_theta = PV("2bmb:TomoScan:NTheta")          # numero totale proiezioni  NUOVO PV
pv_K       = PV("2bmb:TomoScan:KLoops")         # numero di loop interlacciati  NUOVO PV

N_theta = int(pv_N_theta.get()) or 32  # numero totale proiezioni 
K       = int(pv_K.get()) or 4          # numero di loop interlacciati 

# ----------------------------------------------------
# EPICS PVs per Taxi e PSO
# ----------------------------------------------------
pv_start_taxi = PV("2bmb:TomoScan:PSOStartTaxi")         # Posizione di inizio taxi [deg]
pv_end_taxi   = PV("2bmb:TomoScan:PSOEndTaxi")           # Posizione di fine taxi [deg]
pv_counts     = PV("2bmb:TomoScan:PSOCountsPerRotation") # Numero di impulsi per giro del PSO

# Lettura dai PV
start_taxi     = pv_start_taxi.get()           # es: -0.749939 degimbir
end_taxi       = pv_end_taxi.get()             # es: 0.735 deg
counts_per_rev = pv_counts.get()               # es: 11_840_200 impulsi/giro

# ----------------------------------------------------
# BIT-REVERSAL
# ----------------------------------------------------
def bit_reverse(x, bits):
    """Inverte i bit di x su 'bits' bit"""
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

# ----------------------------------------------------
# FUNZIONE TAXI CORRECTION con theta_corrected angoli di timbir corretti 
# ----------------------------------------------------
def taxi_correct(angles_deg, start_taxi, end_taxi, counts_per_rev):
    """
    Corregge gli angoli TIMBIR considerando l'inizio taxi
    e la fine taxi, e li converte in impulsi PSO
    
    Parametri:
        angles_deg      : array degli angoli TIMBIR [deg]
        start_taxi      : angolo di inizio taxi [deg]
        end_taxi        : angolo di fine taxi [deg]
        counts_per_rev  : impulsi per giro del PSO

    Ritorna:
        pulses_corrected      : array impulsi corretti per il PSO
        pulses_end_corrected  : impulso corrispondente alla fine taxi
        theta_corrected       : angoli TIMBIR corretti per start taxi
        theta_end_corrected   : angolo finale corretto per end taxi
    """
    pulse_per_deg = counts_per_rev / 360.0

    theta_corrected = []
    pulses_corrected = []

    # correzione start taxi-> shift angolare
    
    for theta in angles_deg:
        theta_corr = theta + abs(start_taxi)
        theta_corrected.append(theta_corr)
        pulses_corrected.append(theta_corr * pulse_per_deg)

    # correzione fine taxi
    theta_end_corrected = 180.0 + end_taxi
    pulses_end_corrected = theta_end_corrected * pulse_per_deg

    return np.array(pulses_corrected, dtype=int), int(pulses_end_corrected), theta_corrected, theta_end_corrected

# ----------------------------------------------------
# Applico la correzione taxi
# ----------------------------------------------------
pulses_corrected, pulses_end_corrected, theta_corrected, theta_end_corrected = taxi_correct(
    angles_timbir, start_taxi, end_taxi, counts_per_rev
)

# ----------------------------------------------------
# Restituisce gli impulsi reali 
# ----------------------------------------------------
import numpy as np

def compute_real_timeline(theta_corrected, omega_target, accel, counts_per_rev):
    """
    Restituisce gli impulsi reali simulando la rampa del PSO
    θ is in degrees
    theta_correct = angoli TIMBIR corretti per start taxi
    omega_target = velocita' angolare del rotary stage [deg/s] (= cost che il motore raggiunge dopo la fase di accelerazione)
    counts_per_rev = numero di impulsi del PSO per giro completo: quanti impulsi genera il motore per fare 360°

    Durante la rampa la v cresce da 0 a omega_target usando la   θ = 1/2 * a t^2  →  t = sqrt(2θ/a)
    """
   # pv_omega_target = PV("2bmb:TomoScan:rotarystageplateau") 
   # omega_target     = pv_omega_target.get() 
    
    theta_corrected = np.array(theta_corrected)
    omega_target = 500 * 6  # gradi/sec

    # Calcola tempo necessario per accelerare 
    t_acc = omega_target / accel
    theta_acc = 0.5 * accel * t_acc**2

    pulses_per_deg = counts_per_rev / 360.0

    real_pulses = []

    for theta in theta_corrected:

        # Caso 1: angolo cade dentro la rampa di accelerazione
        if theta <= theta_acc:
            # θ = ½ a t^2  →  t = sqrt(2θ/a)
            t = np.sqrt(2 * theta / accel)

        # Caso 2: angolo cade in regime costante
        else:
            t_const = (theta - theta_acc) / omega_target
            t = t_acc + t_const

        # Il PSO genera impulsi a frequenza istantanea
        # In regime costante: pulses = θ * pulses_per_deg
        # In rampa: devi convertirlo come θ(t)
        pulses = theta * pulses_per_deg
        real_pulses.append(int(np.round(pulses)))

    return np.array(real_pulses)



# ----------------------------------------------------
# FUNZIONE DI CONVERSIONE ANGOLI CORRETTI -> IMPULSI PSO : converte solo in impulsi, mantenendo la correzione del taxi già applicata
# ----------------------------------------------------
def angles_corrected_to_pulses_epics(theta_corrected):
    """
    Converte un array di angoli TIMBIR corretti per il taxi (in gradi)
    in impulsi PSO leggendo PSOCountsPerRotation dal PV EPICS.

    Parametri:
        theta_corrected : array/lista di angoli TIMBIR corretti [deg]

    Ritorna:
        pulses : array di impulsi PSO interi
    """
    # Lettura numero di impulsi per giro dal PV
    counts_per_rev = float(pv_counts.get())

    # Assicuro che sia array NumPy
    theta_corrected = np.array(theta_corrected)

    # Conversione angoli -> impulsi
    pulses = theta_corrected * (counts_per_rev / 360.0)

    # Arrotondo e converto in interi
    return np.round(pulses).astype(int)

# ----------------------------------------------------

