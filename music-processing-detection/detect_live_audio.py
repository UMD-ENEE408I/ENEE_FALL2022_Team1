import pyaudio
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from scipy import signal
import signal
import time

# Correlates 15 seconds of song.
# Hit Ctrl-C to see correlation over entire recording period.

def detect_song():
    # Signal handler to show correlation plot.
    def signal_handler(sig, frame):
        plt.plot(corr_history)
        plt.show()
    signal.signal(signal.SIGINT, signal_handler)

    # Import saved audio file and compare to live song, looking for start of song.

    SONG_DATA = "song_data.npy" # Numpy file for song to be detected
    CHUNK = 350 # Number of samples per chunk of data
    DURATION = 15.0 # Number of seconds to look for
    SAMPLE_FORMAT = pyaudio.paFloat32
    CHANNELS = 2 # Number of audio channels
    FS = 22050
    RUNTIME = 10 * 60 # Runtime, in seconds

    p = pyaudio.PyAudio() # Create an interface to PortAudio
    print("===== NOW RECORDING [Press Ctrl+C to debug correlation] =====")
    
    # Open a new stream
    stream = p.open(
        format=SAMPLE_FORMAT,
        channels=CHANNELS,
        rate=FS,
        frames_per_buffer=CHUNK,
        input=True
    )

    song_data = np.load(SONG_DATA) # Get the normalized song data

    # There are 63 chunks per second. We keep data for over 20 seconds in array
    # (20 * 63 = 1260).
    BUFF_LEN = 1500
    data = np.zeros(BUFF_LEN) # Stores max values of each chunk
    # Each entry stores squared norm of previous 945 elements (15 seconds is 945 chunks)
    sq_norms = np.zeros(BUFF_LEN)
    # We also keep a sum of the past 945 entries (to get the average)
    sums = np.zeros(BUFF_LEN)

    # Keeps normalized correlation data over the whole period. Starts with 944 zeros
    # since no correlation is done ending at those 944 values.
    corr_history = np.zeros(944)

    match_found = False # Becomes true when song match is found
    
    # Iterate for runtime defined above
    for i in range(0, int(FS / CHUNK * RUNTIME)):
        # Get chunk data and find its max
        chunk = np.frombuffer(stream.read(CHUNK), dtype=np.float32)
        max = np.amax(chunk)
        data = np.append(data, max) # Then add to data
        data = np.delete(data, 0) # Delete first array element

        # Update sum vector
        sum = sums[BUFF_LEN - 1] + max - data[BUFF_LEN - 945]
        sums = np.append(sums, sum)
        sums = np.delete(sums, 0)

        # Compute squared norm of new max along with previous 629 values
        sq_norm = sq_norms[BUFF_LEN - 1] + np.square(max) - np.square(data[BUFF_LEN - 945])
        # Add it to array
        sq_norms = np.append(sq_norms, sq_norm)
        sq_norms = np.delete(sq_norms, 0)

        # Every 5 seconds starting at 15, run correlation algorithm
        if i % (5 * 63) == 0 and i >= 15 * 63:
            norms = np.sqrt(sq_norms) # Get norms from squared norms
            avgs = sums / 945 # Get avgs from sums

            # Use correlation to dot product saved song with each subsequence of
            # length 945, turning off zero-padding.
            corr = sp.signal.correlate(data, song_data, mode='valid')

            # Normalized correlation - dot product divided by product of norms.
            # corr has 1500 - 945 + 1 = 556 elements. We divide each by the norm of
            # the corresponding set of max values.
            c = np.divide(corr, norms[944:BUFF_LEN])
            
            # Get max correlation value, looking only at values for which avg is
            # greater than some threshold to ignore correlations with noise
            THRESH = 0.015 # Threshold for avg sound
            # Part of avgs corresponding to indices in c (make a copy)
            filter = np.copy(avgs[944:BUFF_LEN])
            filter[filter > THRESH] = 1
            filter[filter <= THRESH] = 0
            c_new = np.multiply(filter, c)

            # On the first iteration, add 1500 - 944 = 556 values to history.
            # For other iterations, only add the 5 * 63 = 315 new ones.
            if i == 15 * 63:
                corr_history = np.concatenate((corr_history, c_new))
            else:
                len_c_new = len(c_new)
                corr_history = np.concatenate((corr_history, c_new[(len_c_new - 5 * 63):len_c_new]))
            
            max_c = np.amax(c_new) # Get max value that was not filtered out
            max_i = np.argmax(c_new) # Index of max correlation
            print("Max correlation value:", max_c) # Print max correlation value
            print("Index of max val:", max_i) # Print its index

            # Now figure out time in song based on correlation vals. Assume that
            # correct correlation is > LIM. First frame with this correlation, use
            # max index and convert back to time.
            LIM = 0.64 # Recognize as song match - adjust this based on song, volume, mic
            if max_c > LIM and not match_found:
                t0 = time.time()
                match_found = True
                # Max index is from last 556 elts, within buffer of length 1500.
                # Number of iterations since max is 556 - max_i. Each i is 1/63
                # of a second, so time since then is (556 - max_i) / 63.
                t_since_detection = (556 - max_i) / 63

                # Clean up, accounting for time to do so. Return time elapsed
                # since max was detected.
                stream.stop_stream()
                stream.close()
                p.terminate()
                t1 = time.time()

                return (t1 - t0) + t_since_detection

    # Stop and close the stream and PyAudio
    stream.stop_stream()
    stream.close()
    p.terminate()

    print("===== FINISHED RECORDING =====")
