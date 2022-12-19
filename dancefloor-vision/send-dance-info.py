import os, subprocess

# Sends move, params to robot.

# Info and location to send
beat = 1.23
move = 2
robot_ip = "192.168.2.148"
robot_port = "2390"

msg = f'{beat},{move}' # Combine into message

# Syscall to send it
print("Sending:", msg)
cmd = "echo -n {} | nc -u {} {}".format(msg, robot_ip, robot_port)
ps = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)