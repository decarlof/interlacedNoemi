"""
theta_interlaced           = angoli TIMBIR ordinati
theta_real_interlaced      = angoli TIMBIR corretti dal taxi model
pulses_interlaced_ideal    = conversione diretta
pulses_interlaced_real     = conversione taxi-corretta

"""
import numpy as np
import math
import matplotlib.pyplot as plt

# ============================================================================
#                       TOMOSCAN OFFLINE (SOLO TAXI)
# ============================================================================
# Questa classe NON genera più gli angoli equispaziati (theta_classic).
# Serve SOLO per:
#   - calcolare il passo angolare corretto secondo l'hardware
#   - calcolare il taxi di inizio e fine scan
#   - mantenere compatibilità con la logica di tomoscan senza EPICS.
# ============================================================================

class TomoScanOffline:

    def __init__(self,
                 rotation_start=0.0,
                 rotation_stop=360.0,
                 num_angles=32,
                 PSOCountsPerRotation=20000,
                 RotationDirection=0,
                 RotationAccelTime=0.15,
                 exposure=0.01,
                 readout=0.01,
                 readout_margin=1):

        # Parametri hardware
        self.PSOCountsPerRotation = PSOCountsPerRotation
        self.RotationDirection    = RotationDirection   # direzione del motore
        self.RotationAccelTime    = RotationAccelTime   # tempo accelerazione
        
        # Parametri della scansione
        self.rotation_start = rotation_start
        self.rotation_stop  = rotation_stop
        self.num_angles     = num_angles
        self.exposure       = exposure
        self.readout        = readout
        self.readout_margin = readout_margin

        # Step iniziale (poi verrà corretto a multiplo dei counts encoder)
        self.rotation_step = (rotation_stop - rotation_start) / (num_angles - 1)

    # ------------------------------------------------------------------------
    def compute_senses(self):
        """
        Calcola la direzione effettiva del moto e degli impulsi encoder.
        Identico a tomoscan, ma senza EPICS.
        """
        encoder_dir = 1 if self.PSOCountsPerRotation > 0 else -1
        motor_dir   = 1 if self.RotationDirection == 0 else -1
        user_dir    = 1 if self.rotation_stop > self.rotation_start else -1
        return encoder_dir * motor_dir * user_dir, user_dir

    # ------------------------------------------------------------------------
    def compute_frame_time(self):
        """Tempo totale per frame = esposizione + readout."""
        return self.exposure + self.readout

    # ------------------------------------------------------------------------
    def compute_positions_PSO(self):
        """
        Calcola le posizioni taxi di inizio/fine scan, come farebbe tomoscan.
        NON produce theta_classic (non serve in pipeline TIMBIR).
        """
        overall_sense, user_direction = self.compute_senses()
        encoder_multiply = self.PSOCountsPerRotation / 360.0

        # Conversione dello step angolare in impulsi encoder → arrotondato a intero
        raw_delta = self.rotation_step * encoder_multiply
        delta_counts = round(raw_delta)

        # Nuovo step angolare "fisico"
        self.rotation_step = delta_counts / encoder_multiply

        # Velocità motore reale
        self.motor_speed = abs(self.rotation_step) / self.compute_frame_time()

        # Distanza necessaria per accelerare (in gradi)
        accel_dist = 0.5 * self.motor_speed * self.RotationAccelTime

        # Punto reale di start
        if overall_sense > 0:
            self.rotation_start_new = self.rotation_start
        else:
            # correzione usata da tomoscan per compatibilità direzioni
            self.rotation_start_new = self.rotation_start - (2 - self.readout_margin)*self.rotation_step

        # Taxi = multiplo intero dello step
        taxi_steps = math.ceil((accel_dist / abs(self.rotation_step)) + 0.5)
        taxi_dist  = taxi_steps * abs(self.rotation_step)

        # Taxi iniziale e finale (in gradi)
        self.rotation_start_taxi = self.rotation_start_new - taxi_dist*user_direction
        self.rotation_stop_new   = self.rotation_start_new + (self.num_angles - 1)*self.rotation_step
        self.rotation_stop_taxi  = self.rotation_stop_new + taxi_dist*user_direction

        return self.rotation_start_taxi, self.rotation_stop_taxi


# ============================================================================
#                       ANGOLI TIMBIR INTERLACED
# ============================================================================
# Usa la tecnica bit-reversal per generare la sequenza interlacciata.
# ============================================================================
    
def bit_reverse(n, bits):
    """Inversione dei bit: es. 0011 → 1100."""
    b = f'{n:0{bits}b}'
    return int(b[::-1], 2)

def generate_timbir_angles(N_theta, K):
    """
    Genera gli angoli TIMBIR usando il bit-reversal.
    Gli angoli NON sono ordinati.
    """
    bits = int(np.log2(N_theta))
    return np.array([bit_reverse(n, bits) * 360.0 / N_theta for n in range(N_theta)])


# ============================================================================
#                       TAXI MODEL DINAMICO
# ============================================================================
# Simula il moto reale dello stage:
#   - accelerazione
#   - plateau a velocità costante
#   - decelerazione
#
# Permette di trovare l’angolo REALE raggiunto al tempo corretto.
# ============================================================================

def simulate_taxi_motion(accel, decel, omega_target, theta_total=360.0, dt=1e-4):

    # Accelerazione: θ(t) = ½ a t²
    T_acc = omega_target / accel
    t_acc = np.arange(0, T_acc, dt)
    theta_acc = 0.5 * accel * t_acc**2

    # Plateau (velocità costante): θ(t) lineare
    theta_flat_len = theta_total - 2 * theta_acc[-1]
    T_flat = theta_flat_len / omega_target
    t_flat = np.arange(0, T_flat, dt)
    theta_flat = theta_acc[-1] + omega_target * t_flat

    # Decelerazione: simmetrica
    T_dec = omega_target / decel
    t_dec = np.arange(0, T_dec, dt)
    theta_dec = theta_flat[-1] + omega_target*t_dec - 0.5*decel*t_dec**2

    # Vettori completi
    t_vec = np.concatenate([t_acc, t_acc[-1]+t_flat, t_acc[-1]+t_flat[-1]+t_dec])
    theta_real = np.concatenate([theta_acc, theta_flat, theta_dec])

    return t_vec, theta_real


def invert_theta(theta_real, t_vec, theta_targets):
    """
    Inverte la funzione θ(t) → t(θ).
    Restituisce il tempo in cui lo stage raggiunge ogni angolo richiesto.
    """
    return np.interp(theta_targets, theta_real, t_vec)


# ============================================================================
#               ANGOLO → IMPULSI ASSOLUTI (per FPGA)
# ============================================================================
    
def convert_to_counts(theta, pulses_per_degree):
    """
    Conversione angolo → impulsi PSO assoluti.
    """
    return np.round(theta * pulses_per_degree).astype(int)


# ============================================================================
#               ★★★★★  PIPELINE COMPLETA  ★★★★★
# ============================================================================
#   - genera θ_interlaced (TIMBIR)
#   - ordina gli angoli in modo crescente
#   - simula taxi per trovare l'angolo reale acquisito
#   - converte angoli → impulsi per FPGA
# ============================================================================

def compute_full_interlaced_pipeline(N_theta, K,
                                     accel=5, decel=5,
                                     omega_target=10,
                                     PSOCountsPerRotation=20000):

    pulses_per_degree = PSOCountsPerRotation / 360.0

    # --------------------------------------------------------
    # 1) Calcolo taxi tramite logica tomoscan
    # --------------------------------------------------------
    ts = TomoScanOffline(num_angles=N_theta,
                         PSOCountsPerRotation=PSOCountsPerRotation)
    taxi_start, taxi_end = ts.compute_positions_PSO()

    # --------------------------------------------------------
    # 2) TIMBIR → ANGOLO INTERLACED
    # --------------------------------------------------------
    theta_interlaced = generate_timbir_angles(N_theta, K)

    # Ordino gli angoli: la pipeline FPGA richiede ordine crescente
    theta_interlaced_sorted = np.sort(theta_interlaced)

    # --------------------------------------------------------
    # 3) Simula il moto reale → θ(t)
    # --------------------------------------------------------
    t_vec, theta_vec = simulate_taxi_motion(accel, decel, omega_target)

    # Trovo il tempo in cui viene raggiunto ogni angolo TIMBIR richiesto
    t_real = invert_theta(theta_vec, t_vec, theta_interlaced_sorted)

    # Angolo reale corrispondente
    theta_interlaced_real = np.interp(t_real, t_vec, theta_vec)

    # --------------------------------------------------------
    # 4) Conversione in impulsi assoluti
    # --------------------------------------------------------
    pulses_interlaced_ideal = convert_to_counts(theta_interlaced_sorted, pulses_per_degree)
    pulses_interlaced_real  = convert_to_counts(theta_interlaced_real,  pulses_per_degree)

    # OUTPUT COMPLETO
    return {
        "theta_interlaced": theta_interlaced_sorted,           # angoli TIMBIR ordinati
        "theta_interlaced_real": theta_interlaced_real,        # angoli taxi-corretti
        "pulses_interlaced_ideal": pulses_interlaced_ideal,    # impulsi ideali
        "pulses_interlaced_real": pulses_interlaced_real,      # impulsi reali per FPGA
        "taxi_start": taxi_start,
        "taxi_end": taxi_end
    }


# ============================================================================
#                                 MAIN TEST
# ============================================================================

if __name__ == "__main__":

    OUT = compute_full_interlaced_pipeline(N_theta=32, K=4)

    print("\n=== ANGOLI INTERLACED (TIMBIR ORDINATI) ===")
    print(OUT["theta_interlaced"])

    print("\n=== IMPULSI IDEALI (senza taxi) ===")
    print(OUT["pulses_interlaced_ideal"])

    print("\n=== IMPULSI REALI (corretti con taxi) ===")
    print(OUT["pulses_interlaced_real"])

