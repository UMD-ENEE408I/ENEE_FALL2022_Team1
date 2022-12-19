import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import librosa

# Takes fft over 5 second windows of saved song.
# See https://realpython.com/python-scipy-fft/.

# Song to use
song_file = "../Lab 11-7-22/sample-audio/message-in-a-bottle.mp3"
y, sr = librosa.load(song_file) # Load audio

len_y = len(y) # Number of samples in song
len_window = 5 * sr # Number of indices in 5 second window

for i in range(0, len_y):
    # Iterate over windows
    if i % len_window == 0 and i > 0:
        window = y[i - len_window:i] # Get 5 second window

        # Take FFT (see link)
        yf = sp.fft.rfft(window)
        xf = sp.fft.rfftfreq(len_window, 1 / sr) # This is in Hz

        # Plot results
        plt.plot(xf, np.abs(yf))
        plt.show()