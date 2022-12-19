import numpy as np
import matplotlib.pyplot as plt
import librosa, librosa.display

# Tracks beat.

def get_beats(song_file, show_plot):
    # First get the signal - song and sample rate.
    y, sr = librosa.load(song_file)

    tempo, beat_times = librosa.beat.beat_track(y=y, sr=sr, start_bpm=60, units='time')
    np.save("tempo.npy", tempo)
    np.save("beat_times.npy", beat_times)

    if show_plot:
        # Plot beats
        plt.figure(figsize=(14, 5))
        librosa.display.waveshow(y, alpha=0.6)
        plt.vlines(beat_times, -1, 1, color='r')
        plt.ylim(-1, 1)
        plt.show()