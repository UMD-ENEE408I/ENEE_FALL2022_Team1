import numpy as np
import matplotlib.pyplot as plt

# Compute weights for intensity throughout song from u, v convolutions.
# Print weights (out of 100), length of weights array, song duration as C++ code.

u = np.load("process_music/u.npy")
v = np.load("process_music/v.npy")
v_full = np.load("process_music/v_full.npy") # Non-absolute value version
dur = np.load("process_music/dur.npy")
beat = np.load("beat.npy")

u_norm = u / np.max(u)
v_norm = v / np.max(v)
# Should go from -1 to +1
v_full_norm = v_full - np.min(v_full)
v_full_norm = v_full_norm / np.max(v_full_norm)
v_full_norm = 2 * (v_full_norm - 0.5)
# Now set a weighted sum and weighted sum with v_full_norm
weighted_sum = 0.7 * u_norm + 0.3 * v_norm
weighted_sum_full = 0.7 * u_norm + 0.3 * v_full_norm
# Don't want any entries that are too small, except for last entries
view = (weighted_sum_full[0:len(weighted_sum_full) - 100])
view[view <= 0.05] = 0.05
# No entries should be 0
weighted_sum_full[weighted_sum_full <= 0] = 0

fig, ax = plt.subplots(5)
ax[0].plot(np.arange(len(u)) * dur / len(u), u_norm)
ax[1].plot(np.arange(len(v)) * dur / len(v), v_norm)
ax[2].plot(np.arange(len(v_full_norm)) * dur / len(v_full_norm), v_full_norm)
ax[3].plot(np.arange(len(v)) * dur / len(v), weighted_sum)
ax[4].plot(np.arange(len(v)) * dur / len(v), weighted_sum_full)
plt.show()

print("float weights[] = {", end="")
for i in range(0, len(weighted_sum_full)):
    str = format(np.round(weighted_sum_full[i] * 100).astype(np.int32))
    print(str, end="")
    if i < len(weighted_sum_full) - 1:
        print(",", end="")
print("};")
print(f"int len_weights = {len(weighted_sum_full)};")
print(f"float song_dur = {'{:.2f}'.format(dur)};")
print(f"float beat_pd = {'{:.2f}'.format(beat)};")