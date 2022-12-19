import numpy as np
import matplotlib.pyplot as plt
import librosa

# Reads first few seconds from saved signal. Splits into chunks, takes maximum
# of each signal, and saves normalized result to file. Sample rate should be
# 22050, otherwise song can be resampled.

SONG = "../../sample-audio/defying-gravity.mp3" # Song file location
OFFSET = 0.0 # Offset from start of song
DURATION = 15.0 # Number of seconds to use
CHUNK = 350 # Number of frames per chunk

# First get the signal - song and sample rate.
y, sr = librosa.load(SONG, offset=OFFSET, duration=DURATION)
print("Sample rate:", sr) # Should be 22050

# Crop array so number length is divisible by chunk size
old_len = len(y)
new_len = old_len - (old_len % CHUNK)
y = y[0:new_len]

# Reshape array to form chunks
y = np.reshape(y, (-1, CHUNK)) # -1 computes missing dimension automatically

# Now find max from each chunk and then normalize data.
data = np.amax(y, axis=1)
data = data / np.linalg.norm(data)
print("Data length:", len(data))

# Save the audio data
np.save("song_data.npy", data)

# Display song data as plot
plt.plot(data)
plt.show()