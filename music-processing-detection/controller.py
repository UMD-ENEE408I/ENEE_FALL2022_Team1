import numpy as np
import subprocess
import time
import signal

# Other script
import detect_live_audio

# STEPS:
# 1. Run get_saved_audio.py to prepare live song detection.
# 2. Run process_music_final.py to process music, generate dance moves.
# 3. Run compute_weights.py and paste output into robot cpp code.
# 4. Run controller.py to wait for music to start, then send the dance moves.
# 5. Update song_detection_error below

song_detection_error = -2

# Signal handler to quit controller loop
def signal_handler(sig, frame):
    quit()

# Robot networking info [Leo, Melissa, Brandon]
robot_ips = ["192.168.22.126"]#, "192.168.2.104", "192.168.2.134"]
robot_ports = ["2390"]#, "2390", "2390"]

# Load dance move info from music processing.
segment_times = np.load("segment_times.npy")
segment_durs = np.load("segment_durs.npy")
beat = np.around(np.load("beat.npy"), 3)
mv_types = np.load("mv_types.npy")
mv_nums = np.load("mv_nums.npy")
mv_params = np.load("mv_params.npy")

# Call script to listen for song start.
t_since = detect_live_audio.detect_song() # Returns time since detection in s
t_since = t_since + 15 - song_detection_error # To get time sing song start
t0 = time.time() # Time after function call

# Bind signal now to overwrite other handler
signal.signal(signal.SIGINT, signal_handler)

# Controller loops, sends data every two seconds
print("===== CONTROLLER LOOP [Press Ctrl+C to quit] =====")
i_old = -1 # Index of previous segment
mv = 0 # Current move number
param = 0 # Current param
while True:
    t = time.time() - t0 + t_since # Get time since song start
    # If last entry (end of song) is less than time, quit
    if ((segment_times < t)[len(segment_times) - 1]):
        print("Done!")
        quit()
    i = np.argmin(segment_times < t) - 1 # Index of current segment

    # If switched to new segment
    if i != i_old:
        print("Time since song start:", t, flush=True)
        print("Segment #", i)
        i_old = i

        # Get mv number, param
        mv = mv_nums[i].astype(np.int32) # Get move number
        param = beat * 2 # Get param to go with it. Send 2x beat for short moves.
        if mv_types[i] == 1:
            # Long moves have number 1, 2, 3, 4
            mv = mv + 1
            param = np.around(mv_params[i], 3)
            # For longer move, increase period
            param = param + 12
            if mv == 4:
                param = param + 7
        else:
            mv += 10 # Numbers 10, 11

        msg = f"30 {np.around(t, 3)} {mv} {param} 1" # For long moves
        if mv_types[i] == 0: # If short move
            msg = f"{mv} {param}"

        # Send data to robots, 4 times
        for j in range(0, 4):
            # Syscall to send it
            print("Sending:", msg)
            # Send to 3 robots
            for k in range(0, 1):
                cmd = "echo -n {} | nc -u {} {}".format(msg, robot_ips[k], robot_ports[k])
                ps = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            
            time.sleep(1)

    time.sleep(0.1) # Wait for a bit
