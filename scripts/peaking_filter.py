
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt



# Define our constants
f_s = 48000.0 # Sampling Frequency
T = 1.0 / f_s # Sampling Period
f_c = 10000.0 # Center Frequency of Peaking Filter
bw = 1000.0 # Bandwidth of Peaking Filter
G = 1 # Gain of Peaking Filter (G = 1 -> All Pass, 0 < G < 1 -> Cut, G > 1 -> Boost)
Q = f_c / bw # Quality Factor of Peaking Filter
w_ct = 2.0 * np.tan(np.pi * f_c * T) # Substitution term to make coefficient calculation eaiser


# Calculate the filter coefficients (These coefficients assume that the difference equation is in standard unnormalized Direct Form I)
a = [0.0, 0.0, 0.0] # Ouput or "y" coefficients
b = [0.0, 0.0, 0.0] # Input or "x" coefficients

a[0] = 4.0 + ((2.0/Q)*w_ct) + w_ct**2
a[1] = (2.0 * w_ct**2) - 8.0
a[2] = 4.0 - ((2.0/Q)*w_ct) + w_ct**2

b[0] = 4.0 + ((2.0*G/Q)*w_ct) + w_ct**2
b[1] = (2.0 * w_ct**2) - 8.0
b[2] = 4.0 - ((2.0*G/Q)*w_ct) + w_ct**2

np.set_printoptions(legacy='1.25') # For prettier printing
print("a:", a)
print("b:", b)

w, h = signal.freqz(b, a) # Calculate filter points based on the coefficients


# Plot the digital filter
fig, ax1 = plt.subplots()

ax1.set_title("Peaking Filter Frequency Response")
if G == 1.0: # Stop autoscaling for the All Pass case since the magnitude response fluctuates with very small amounts around 0 dB
    ax1.autoscale(False) 
ax1.plot(w, 20 * np.log10(abs(h)), "b")
ax1.set_ylabel("Amplitude [dB]", color="b")
ax1.set_xlabel("Frequency [rad/sample]")

ax2 = ax1.twinx()
angles = np.unwrap(np.angle(h))
if G == 1.0: # Similar case with the magnitude response above
    ax2.autoscale(False)
ax2.plot(w, angles, "g")
ax2.set_ylabel("Angle (radians)", color="g")
ax2.grid(True)
ax2.axis("tight")

plt.show()
