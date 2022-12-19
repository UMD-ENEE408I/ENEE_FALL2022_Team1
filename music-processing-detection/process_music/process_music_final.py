import numpy as np

# Other scripts
import plot_segmentation
import beat_track
import convolve

# Process a piece of music: segmentation, beat, loudness/variation.

alreadyProcessed = False # Whether processing data already saved in folder

# Audio file
# Note: if getting weird audio error with librosa.load(), try different .mp3
# or pass explicit duration argument.
song_file = "../../../sample-audio/shake-it-off.mp3"

if not alreadyProcessed:
    print("===== Getting segmentation =====")
    plot_segmentation.compute_segments(song_file, True)

    print("===== Beat tracking =====")
    beat_track.get_beats(song_file, True)

    print("===== Convolution =====")
    convolve.convolve_song(song_file, True)

# Now load in all the data
segment_times = np.load("segment_times.npy")
segment_nums = np.load("segment_nums.npy")
tempo = np.load("tempo.npy")
beat_times = np.load("beat_times.npy")
u = np.load("u.npy") # Loudness
v = np.load("v.npy") # Variation
dur = np.load("dur.npy") # Song length in s

print("===== Dance move assignment =====")

# Now adjust the segments to make them the right length, and to deal with fact
# that dancing only starts after song detected (after 15 seconds).
# Eliminate all segment starts before 20 seconds.
for t in segment_times:
    if t < 20:
        segment_times = np.delete(segment_times, 0)
        segment_nums = np.delete(segment_nums, 0)
# Now set first index time to 20, regardless of what it is
segment_times[0] = 20

for j in range(0, 2):
    # Next, eliminate any short segment by merging with short segment next to it.
    # Setting min length to 12 seconds.
    i = 1
    while i < len(segment_times):
        segment_dur = segment_times[i] - segment_times[i - 1]
        if segment_dur < 8:
            # Get duration of previous, next segments to determine which to merge with
            dur_prev = 10000
            dur_next = 10000
            if i > 1:
                dur_prev = segment_times[i - 1] - segment_times[i - 2]
            if i < len(segment_times) - 1:
                dur_next = segment_times[i + 1] - segment_times[i]
            if dur_prev < dur_next:
                # Merge with previous
                segment_times = np.delete(segment_times, i - 1)
                segment_nums = np.delete(segment_nums, i - 1)
            else:
                # Merge with next
                segment_times = np.delete(segment_times, i)
                segment_nums = np.delete(segment_nums, i - 1)
        else:
            i = i + 1 # Go to next segment

# Next, break up any segments that are too long using maximums in the variation.
i = 1
while i < len(segment_times):
    segment_dur = segment_times[i] - segment_times[i - 1]
    corresp_num = segment_nums[i - 1] # Corresponding segment number
    if segment_dur > 22:
        # Window within which to insert new transition
        window_start = segment_times[i - 1] + segment_dur / 4
        window_end = segment_times[i] - segment_dur / 4

        # Corresponding indices in v
        start_i = np.rint((window_start / dur) * len(v)).astype(np.int32)
        end_i = np.rint((window_end / dur) * len(v)).astype(np.int32)

        # Get index of max variation, convert to time
        max_i = np.argmax(v[start_i:end_i])
        max_t = ((max_i + start_i)/ len(v)) * dur

        # Insert time, num into arrays
        segment_times = np.insert(segment_times, i, max_t)
        segment_nums = np.insert(segment_nums, i - 1, corresp_num)

        i = i + 2 # Since added something before
    else:
        i = i + 1

# Compute average intensity and loudness for each segment
avgs_u = np.array([]) # Average loudness
avgs_v = np.array([]) # Average variation
segment_durs = np.array([]) # Segment durations
for i in range(1, len(segment_times)):
    segment_dur = segment_times[i] - segment_times[i - 1]
    segment_durs = np.append(segment_durs, segment_dur)

    # Start, end indices in u/v
    start_i = np.rint((segment_times[i - 1] / dur) * len(u)).astype(np.int32)
    end_i = np.rint((segment_times[i] / dur) * len(u)).astype(np.int32)

    # Get avg loudness, variation
    avg_u = np.average(u[start_i:end_i])
    avg_v = np.average(v[start_i:end_i])

    # Add averages to list
    avgs_u = np.append(avgs_u, avg_u)
    avgs_v = np.append(avgs_v, avg_v)

# Assign dance moves to segments.
# Rules: shortest segments get back/forth moves (short 0, 1). These are parametrized
# by tempo/beat. We alternate between 0 and 1.
# Longer segments get trajectories (long 0, 1, 2). These are parametrized by
# weighted loudness and variation.
# We want to alternate between both types and include several instances of both.
# Currently not using segment_nums.
num_segments = len(segment_durs) # Number of segments
num_short = np.floor(num_segments / 2).astype(np.int32)

mv_types = np.ones(num_segments) # 0 if short move, 1 if long
# Partition to get num_short smallest then remaining larger values
part = np.argpartition(segment_durs, num_short)
for i in range(0, num_short):
    mv_types[part[i]] = 0

# Now assign move numbers, cycling for large and small
mv_nums = np.array([])
next_short = 0 # Next short dance move to be assigned
next_long = 0 # Next long dance move to be assigned
for i in range(0, num_segments):
    # If short segment
    if mv_types[i] == 0:
        mv_nums = np.append(mv_nums, next_short)
        next_short = (next_short + 1) % 2 # Update next short
    # Else if long segment
    else:
        mv_nums = np.append(mv_nums, next_long)
        next_long = (next_long + 1) % 4 # Update next long

# Finally get params for long moves from avg loudness/variation. (Although just
# compute param for every move - this is simpler.)
# Normalize u, v linearly (so they have values from 0 to 1)
u_norm = avgs_u - np.min(avgs_u)
u_norm = u_norm / np.max(u_norm)
v_norm = avgs_v - np.min(avgs_v)
v_norm = v_norm / np.max(v_norm)
# Compute params as weighted average, with focus on v
mv_params = np.array([])
for i in range(0, num_segments):
    weighted_avg = 0.2 * u_norm[i] + 0.8 * v_norm[i] # From 0 to 1
    # This is period for dance move; subtract song intensity from it (more
    # intense dance moves go faster).
    param = 8 - 4 * weighted_avg
    mv_params = np.append(mv_params, param)

beat = 60 / tempo # Beat period - param for short moves

# Print the computed schedule, then save the information.
print("Segment times:", segment_times)
print("Segment durations:", segment_durs)
print("Avg loudness:", avgs_u)
print("Avg variation:", avgs_v)
print("Beat period:", beat)
print("Move types:", mv_types)
print("Move numbers:", mv_nums)
print("Move params:", mv_params)

# Save info needed for dance
# Note - these are different from segment times in current folder
np.save("../segment_times.npy", segment_times)
np.save("../segment_durs.npy", segment_durs)
np.save("../beat.npy", beat)
np.save("../mv_types.npy", mv_types)
np.save("../mv_nums.npy", mv_nums)
np.save("../mv_params.npy", mv_params)