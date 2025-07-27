from scipy import signal

# Design IIR Butterworth 2nd order LPF
fs = 10000  # Hz
fc = 1000   # cutoff freq

sos = signal.butter(N=2, Wn=fc/(fs/2), btype='low', output='sos')
print(sos)

# [[ 0.06745527  0.13491055  0.06745527  1.         -1.1429805   0.4128016 ]]