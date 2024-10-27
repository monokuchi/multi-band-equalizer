
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt



# Define our constants
NUM_FILTERS = 10 # Total number of Peaking Filters
FFT_SIZE = 2**16 # Size of FFT taken in freqz()

f_s = 48000.0 # Sampling Frequency [Hz]
T = 1.0 / f_s # Sampling Period [Sec]
f_c = np.array([30.0, 60.0, 120.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0, 12800.0]) # Center Frequencies of Peaking Filters [Hz]
bw = np.array([5.0, 10.0, 20.0, 40.0, 100.0, 145.0, 240.0, 500.0, 1200.0, 6000.0]) # Bandwidths of Peaking Filters [Hz]
G = np.array([2.0, 2.0, 2.0, 2.0, 0.5, 2.0, 0.5, 2.0, 2.0, 0.5]) # Gains of Peaking Filters (G = 1 -> All Pass, 0 < G < 1 -> Cut, G > 1 -> Boost)
assert (len(f_c) == NUM_FILTERS and len(bw) == NUM_FILTERS and len(G) == NUM_FILTERS)
Q = f_c / bw # Quality Factors of Peaking Filters
w_ct = 2.0 * np.tan(np.pi * f_c * T) # Substitution terms to make coefficient calculation eaiser


# Calculate the filter coefficients (These coefficients assume that the difference equation is in standard unnormalized Direct Form I)
A = np.zeros((NUM_FILTERS, 3)) # Array of ouput or "y" coefficients
B = np.zeros((NUM_FILTERS, 3)) # Array of input or "x" coefficients
W = np.zeros(FFT_SIZE) # Array of frequencies which correspond to H
H = np.complex128(FFT_SIZE) # Array of the cascaded frequency response

for i in range(NUM_FILTERS):
    A[i][0] = 4.0 + ((2.0/Q[i])*w_ct[i]) + w_ct[i]**2
    A[i][1] = (2.0 * w_ct[i]**2) - 8.0
    A[i][2] = 4.0 - ((2.0/Q[i])*w_ct[i]) + w_ct[i]**2

    B[i][0] = 4.0 + ((2.0*G[i]/Q[i])*w_ct[i]) + w_ct[i]**2
    B[i][1] = (2.0 * w_ct[i]**2) - 8.0
    B[i][2] = 4.0 - ((2.0*G[i]/Q[i])*w_ct[i]) + w_ct[i]**2

    w, h = signal.freqz(B[i], A[i], worN=FFT_SIZE, fs=f_s) # Calculate filter points based on the coefficients
    W = w
    H += h

np.set_printoptions(legacy='1.25') # For prettier printing
print("A:", A)
print("B:", B)



# Plot the digital filter
fig, ax1 = plt.subplots()

ax1.set_title("Peaking Filter Frequency Response")
if np.all(G == 1.0): # Stop autoscaling for the All Pass case since the magnitude response fluctuates with very small amounts around 0 dB
    ax1.autoscale(False) 
ax1.plot(W, 20 * np.log10(abs(H)), "b")
ax1.set_ylabel("Amplitude [dB]", color="b")
ax1.set_xlabel("Frequency [Hz]")

ax2 = ax1.twinx()
angles = np.unwrap(np.angle(H))
if np.all(G == 1.0): # Similar case with the magnitude response above
    ax2.autoscale(False)
ax2.plot(W, angles, "g")
ax2.set_ylabel("Angle (radians)", color="g")
ax2.grid(True)
ax2.axis("tight")

plt.show()
