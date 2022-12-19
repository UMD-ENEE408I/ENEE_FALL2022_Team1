import numpy as np
import matplotlib.pyplot as plt
import librosa, librosa.display

# Remove beats, then do convolutions to get loudness, variation.
# Tempo and beat times should already be saved locally.

def convolve_song(song_file, show_outputs):
    # First get the signal - song and sample rate.
    y, sr = librosa.load(song_file)

    dur = len(y) / sr # Duration of song, in s

    # REMOVE BEATS

    # Will remove beats from y_new
    y_new = np.copy(y)

    # First, track the beat
    tempo, beat_times = librosa.beat.beat_track(y=y, sr=sr, start_bpm=60, units='time')
    beat_pd = (60.0 * sr / tempo) # Beat period in # samples
    # Approximate number of samples in beat sound itself (1/5th of period)
    beat_len = np.rint(beat_pd / 5).astype(np.int32)

    # For each beat in audio
    for beat_time in beat_times:
        # Index of start of beat in samples
        start_i = np.rint((beat_time - 0.07) * sr).astype(np.int32)

        # Starting at this index, flatten down range of size beat_len
        beat_max = np.amax(y[start_i:start_i + beat_len]) # Max val in beat
        prev = np.amax(y[start_i - beat_len:start_i]) # Volume before beat
        next = np.amax(y[start_i + beat_len:start_i + 2 * beat_len]) # Volume after beat
        # Squash down beat
        for i in range(beat_len):
            y_new[start_i + i] = ((prev + (i / beat_len) * (next - prev)) * y[start_i + i]) / beat_max

    # UPPER ENVELOPE

    CHUNK = np.rint(sr / 12).astype(np.int32) # 12 chunks per second
    # Crop array so number length is divisible by chunk size
    old_len = len(y_new)
    new_len = old_len - (old_len % CHUNK)
    y_view = y_new[0:new_len]
    # Reshape array to form chunks
    z = np.reshape(y_view, (-1, CHUNK)) # -1 computes missing dimension automatically
    z = np.amax(z, axis=1) # Now find max from each chunk and then normalize data.

    # Plot data with domain in seconds for each
    fig, ax = plt.subplots(3)
    ax[0].set(title="Original song data")
    ax[0].plot(np.arange(len(y)) / sr, y)
    ax[1].set(title="Beats squashed")
    ax[1].plot(np.arange(len(y_new)) / sr, y_new)
    ax[2].set(title="Upper envelope")
    ax[2].plot(np.arange(len(z)) * dur / len(z), z)
    if show_outputs:
        plt.show()

    # CONVOLUTIONS (to smooth out, edge detect)

    # Vectors interpolation dormal dist
    normal_dist_11 = [0.077, 0.194, 0.398, 0.664, 0.903, 1, 0.903, 0.664, 0.398, 0.194, 0.077]
    normal_dist_11 = normal_dist_11 / np.sum(normal_dist_11)
    normal_dist_21 = [0.077, 0.126, 0.194, 0.285, 0.398, 0.527, 0.664, 0.794, 0.903, 0.975, 1, 0.975, 0.903, 0.794, 0.664, 0.527, 0.398, 0.285, 0.194, 0.126, 0.077]
    normal_dist_21 = normal_dist_21 / np.sum(normal_dist_21)

    evp = np.convolve(z, normal_dist_11, mode="same") # Get upper envelope

    # Smooth evp for loudness
    u = np.convolve(evp, normal_dist_21, mode="same")
    # Slope/edge detection
    v = np.convolve(evp, np.concatenate((np.ones(8), np.zeros(6), -1 * np.ones(8))), mode="same")
    v_full = v # Non-absolute value version
    v = np.abs(v)

    # Save data
    np.save("u.npy", u)
    np.save("v.npy", v)
    np.save("v_full.npy", v_full)
    np.save("dur.npy", dur)

    if show_outputs:
        # Plot results
        fig, ax = plt.subplots(4)
        ax[0].set(title="Upper envelope")
        ax[0].plot(np.arange(len(z)) * dur / len(z), z)
        ax[1].set(title="Smoothed envelope")
        ax[1].plot(np.arange(len(evp)) * dur / len(evp), evp)
        ax[2].set(title="Loudness")
        ax[2].plot(np.arange(len(u)) * dur / len(u), u)
        ax[3].set(title="Abs(slope) (edge detection)")
        ax[3].plot(np.arange(len(v)) * dur / len(v), v)
        plt.show()
