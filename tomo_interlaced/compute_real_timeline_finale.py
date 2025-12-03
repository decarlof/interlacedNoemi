def compute_real_timeline(theta_corrected, counts_per_rev):
    """
Legge i parametri reali del motore (velocità, accelerazione, risoluzioni…)
Divide la rotazione in tre fasi: accelerazione – plateau – decelerazione
Calcola quanti impulsi PSO x tempo/angolo del trigger
    
    Tutte le velocità sono in °/sec, lette dai PV del rotary stage
    """

    # ------------------------------
    # IMPORT EPICS PV
    # ------------------------------
    
    pv_vmax = PV("2bmb:m102.VMAX")   # velocità massima (plateau)
    pv_velo = PV("2bmb:m102.VELO")   # velocità corrente (debug)
    pv_vbas = PV("2bmb:m102.VBAS")   # velocità base a inizio rampa
    pv_accl = PV("2bmb:m102.ACCL")   # accelerazione (deg/s^2 o tempo ramp)
    pv_mres = PV("2bmb:m102.MRES")   # motor resolution
    pv_eres = PV("2bmb:m102.ERES")   # encoder resolution
    pv_rres = PV("2bmb:m102.RRES")   # readback resolution

    # ------------------------------
    # LETTURA PV
    # ------------------------------
    VELO = pv_velo.get()        # plateau target [deg/s]
    VBAS = pv_vbas.get()        # base velocity [deg/s]
    ACCL = pv_accl.get()        # accelerazione [deg/s^2]
    mres = pv_mres.get()
    eres = pv_eres.get()
    rres = pv_rres.get()

    # ------------------------------
    # Conversione angoli -> impulsi
    # ------------------------------
    pulse_per_deg = counts_per_rev / 360.0
    pulses_timeline = []

    # ------------------------------
    # Calcolo parametri rampa
    # ------------------------------
    # Accelerazione uniforme da VBAS -> VELO
    a_acc = ACCL
    theta_accel = (VELO**2 - VBAS**2) / (2 * a_acc)  # angolo percorso in accelerazione da VBAS a VELO

    # Decelerazione uniforme: simmetrica accelerazione
    a_dec = a_acc
    theta_decel = theta_accel  # simmetrico

    # Plateau
    theta_plateau = 180.0 - theta_accel - theta_decel  # angolo percorso a VELO costante

    # ------------------------------
    # Loop su tutti gli angoli corretti : valuto ogni angolo dove si trova
    # ------------------------------
    for theta in theta_corrected:

        # -----------------------------------------------
        # 1) Fase accelerazione
        # -----------------------------------------------
        if theta <= theta_accel:
            # moto uni accelerato
            t = (np.sqrt(VBAS**2 + 2*a_acc*theta) - VBAS) / a_acc
            pulses = theta * pulse_per_deg
            pulses_timeline.append(int(pulses))
            continue

        # -----------------------------------------------
        # 2) Fase plateau
        # -----------------------------------------------
        elif theta <= (theta_accel + theta_plateau):
            # angolo rimanente dopo accelerazione
            theta_remain = theta - theta_accel
            t_plateau = theta_remain / VELO
            pulses = theta * pulse_per_deg
            pulses_timeline.append(int(pulses))
            continue

        # -----------------------------------------------
        # 3) Fase decelerazione
        # -----------------------------------------------
        else:
            # angolo rimanente da percorrere in decelerazione
            theta_remain = 180.0 - theta
            t_decel = (VELO - np.sqrt(VELO**2 - 2*a_dec*theta_remain)) / a_dec
            pulses = theta * pulse_per_deg
            pulses_timeline.append(int(pulses))
            continue

    return np.array(pulses_timeline, dtype=int)
