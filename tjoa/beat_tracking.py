#matplotlib inline
import numpy, scipy, matplotlib.pyplot as plt, IPython.display as ipd
import librosa, librosa.display
import stanford_mir; stanford_mir.init();

x, sr = librosa.load('../Downloads/audio_125_bounce.wav')

tempo, beat_times = librosa.beat.beat_track(x, sr=sr, start_bpm=60, units='time')
print(tempo)
print(beat_times)

plt.figure(figsize=(14, 5))
librosa.display.waveshow(x, alpha=0.6)
plt.vlines(beat_times, -1, 1, color='r')
plt.ylim(-1, 1)

beat_times_diff = numpy.diff(beat_times)
plt.figure(figsize=(14, 5))
plt.hist(beat_times_diff, bins=50, range=(0,4))
plt.xlabel('Beat Length (seconds)')
plt.ylabel('Count')
