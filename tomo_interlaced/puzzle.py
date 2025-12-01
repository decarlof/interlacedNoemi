import numpy as np

def generate_angles_timbir(N_theta, K):
    """
    Generate TIMBIR-style interlaced angles using bit-reversal ordering.

    Parameters
    ----------
    N_theta : int
        Total number of projections (acquired angles).
    K : int
        Number of interlaced loops (must be a power of 2).

    Returns
    -------
    angles_deg : np.ndarray
        Array of acquisition angles in degrees, mapped in [0, 180).
    loop_idx : np.ndarray
        Array of loop indices (0 .. K-1) to which each angle belongs.
    """
    # ------------------------
    # Bit-reversal helper
    # ------------------------
    def bit_reverse(x, bits):
        b = f'{x:0{bits}b}'   # rappresentazione binaria con bits
        return int(b[::-1], 2)  # inverti la stringa e riconverti in intero

    # numero di bit necessario per rappresentare K loop
    bits = int(np.log2(K))
    angles = []
    loop_indices = []
    # ------------------------
    # Compute acquisition angles in time order
    # ------------------------
    for n in range(N_theta):  # indice base legato al tempo
        base = n * K   # loop di appartenenza (0..K-1)
        
        loop = (base // N_theta) % K  # bit-reversal del loop index
        
        rev = bit_reverse(loop, bits)    # indice finale per l'angolo
      
        val = base + rev  
        
        theta = val * 360.0 / N_theta     # angolo su 360° (TIMBIR originale)
        
        theta = theta % 180.0     # mappo su 0, 180° per tomo

        angles.append(theta)
        loop_indices.append(loop)

    return np.array(angles), np.array(loop_indices)

# Chiamo la funzione 
  angles, loops = generate_angles_timbir(N_theta=32, K=4)

print("Angles:", angles)
print("Loop indices:", loops)

   


  #----------------------------------------------------
  # Prendi angles_deg e converti in impulsi reali  vedi formula su appunti
  #----------------------------------------------------
''' La conversione in inmpulsi prende angoli in gradi e li traforma in  N impulsi dell'encoder  '''





  #----------------------------------------------------
  # Calcola i ritardi tra un impulso e il successivo (memPulseSeq)
  #----------------------------------------------------







