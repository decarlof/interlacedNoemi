"""
La conversione da angolo a impulsi
il numero di impulsi teorico da inviare al rotary N_ideal , theta_ideal e' l'angolo desirato in gradi
N_impulsi impulsi servono per un giro completo

pso_model == 'A3200'
Speed (rpm) 500

Leggere gli impulsi/giro da EPICS (PSOCountsPerRotation)
Parlare in gradi: prendere la stringa di angoli e trasformarla in un array di float
Applicare la formula del file overleaf
Restituire gli impulsi, in un formato pronto per il PSO (lista, numpy array o PV)


(in program_PSO()), devi passare:
lista impulsi
oppure solo start/stop e step (dipende dal tipo di interlaced)


"""
"""
 Converte una lista di angoli (stringa) in impulsi PSO: np.ndarray  di impulsi teorici da inviare al PSO
 - leggere dal controllo EPICS quanti impulsi produce il motore per compiere una rotazione completa di 360°
 - PV PSOCountsPerRotation  impostato dal codice nel costruttore per la A3200 : quanticounts servono al tuo encoder per fare un giro completo?
 - self.epics_pvs['PSOCommand.BOUT'].put("UNITSTOCOUNTS(%s, 360.0)" % pso_axis) da tomoscan_pso (36)
 - converti 360° in counts (impulsi) dell' encoder e mettili nel Buffer OUT (BOUT) del PSO
 - UNITSTOCOUNTS comando Aerotech
 """
# ------------------------------------------------------------
# ------------------------------------------------------------

import numpy as np
from epics import PV

# PV che contiene il numero di impulsi per giro
pso_counts_pv = PV('PSOCountsPerRotation')

# --- FUNZIONE DI CONVERSIONE ---
def angles_to_pulses_epics(angle_string):
    """
    Converte una lista di angoli stringa in impulsi PSO usando i counts EPICS
    """
    #  numero di impulsi per giro dal PV
    counts_per_rev = float(pso_counts_pv.get())

    #  parsing della stringa di angoli
    angles_deg = np.array([float(x) for x in angle_string.split(',')])

    # conversione angoli → impulsi
    pulses = angles_deg * (counts_per_rev / 360.0)

    return pulses



if __name__ == "__main__":
    angles = "0, 0.25, 0.5, 0.75, 1.0"
    pulses = angles_to_pulses_epics(angles)
    print("Angoli:", angles)
    print("Impulsi PSO:", pulses)











"""

 def __init__(self, pv_files, macros):
        super().__init__(pv_files, macros)
        self.epics_pvs['ProgramPSO'].put('Yes') # scrive si probabilmente lo attiva
        # On the A3200 we can read the number of encoder counts per rotation from the controller
        # Unfortunately the Ensemble does not support this
        pso_model = self.epics_pvs['PSOControllerModel'].get(as_string=True) # seleziona modello
        if (pso_model == 'A3200'):
            pso_axis = self.epics_pvs['PSOAxisName'].get(as_string=True)  # se e' il modello A3200
            self.epics_pvs['PSOCommand.BOUT'].put("UNITSTOCOUNTS(%s, 360.0)" % pso_axis, wait=True, timeout=10.0) # scrive un comando sul pv  e units count chiede al controller quanti conteggi corrispondono a 360 gradi
            # a cosa si riferisce il timeout ? alla lettura? latenza?
            reply = self.epics_pvs['PSOCommand.BINP'].get(as_string=True) # risposta controller = richiede una stringa   convertita successivamnente in float
            counts_per_rotation = float(reply[1:])  #Questo numero rappresenta i counts per rotation, cioè quanti “passi” o unità numeriche produce l’encoder in un giro completo
            self.epics_pvs['PSOCountsPerRotation'].put(counts_per_rotation)
        self.epics_pvs['CamUniqueIdMode'].put('Camera',wait=True)

"""










