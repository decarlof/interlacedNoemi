import numpy as np
import matplotlib.pyplot as plt

def bit_reversal_interlaced(N_theta=32, K=4, r_outer=1.0, r_step=0.15):
    def bit_reverse(x, bits):
        return int(f'{x:0{bits}b}'[::-1], 2)

    bits = int(np.log2(K))
    angles = []
    loops = []

    for n in range(N_theta):
        val = n * K + bit_reverse((n * K // N_theta) % K, bits)
        theta = val * 2 * np.pi / N_theta
        angles.append(theta)
        loops.append((n * K // N_theta) % K)

    angles = np.array(angles)
    loops = np.array(loops)
    radii = r_outer - loops * r_step

    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111, polar=True)
    ax.set_title(f"Bit-Reversal Interlacing (N={N_theta}, K={K})")

    ax.plot(angles, radii, '-o', lw=1.2)

    for i in range(N_theta):
        ax.text(angles[i], radii[i] + 0.03, str(loops[i]+1), ha='center')

    ax.set_rticks([])
    plt.show()

    # Additional outputs
    angles_deg = np.degrees(angles)
    print("Angles (deg):")
    print(", ".join(f"{a:.2f}" for a in angles_deg))
