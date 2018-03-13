import numpy as np
import matplotlib.pyplot as plt
from scipy import fft

Fs = 150                         # sampling rate
Ts = 1.0/Fs                      # sampling interval
t = np.arange(0,2,Ts)            # time vector
ff = 5                           # frequency of the signal
y = np.sin(2 * np.pi * ff * t)
print("size of y: " + str(y.size))
print("size of t: " + str(t.size))
print(t)


plt.subplot(2,1,1)
plt.plot(t,y,'k-')
plt.xlabel('time')
plt.ylabel('amplitude')

plt.subplot(2,1,2)
n = len(y)                       # length of the signal
k = np.arange(n)
# print(k)
T = n/Fs
# print(T)
frq = k/T #  sides frequency range
freq = frq[range(n/2)]           # one side frequency range

Y = np.fft.fft(y)/n              # fft computing and normalization
print(Y.size)
Y = Y[range(n/2)]
print(Y.size)

plt.plot(freq, abs(Y), 'r-')
plt.xlabel('freq (Hz)')
plt.ylabel('|Y(freq)|')

plt.show() 