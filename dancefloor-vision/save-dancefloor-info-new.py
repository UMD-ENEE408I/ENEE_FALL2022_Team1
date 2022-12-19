import numpy as np

# Calculate dancefloor info and save to files.
#
#       __40
#  45/      \17    . = (0,0)   y|_x
# 48|        |6
#   .\49__0 /1
#

ft_to_cm = 30.48 * np.array([1, 1, 1]) # Number of cm in a foot

# Calculate transformation matrix for tag 0.
t_tag0_to_wrld = np.transpose(np.multiply(np.array([[4, 0, 0.5]]), ft_to_cm))
R_tag0_to_wrld = np.array([
    [-1,  0,  0],
    [ 0,  0, -1],
    [ 0, -1,  0]
])
T_tag0_to_wrld = np.concatenate((np.concatenate((R_tag0_to_wrld, t_tag0_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 1.
t_tag1_to_wrld = np.transpose(np.multiply(np.array([[6.8284, 1.1716, 0.5]]), ft_to_cm))
R_tag1_to_wrld = np.array([
    [-0.7071,  0,  0.7071],
    [-0.7071,  0, -0.7071],
    [ 0,      -1,  0     ]
])
T_tag1_to_wrld = np.concatenate((np.concatenate((R_tag1_to_wrld, t_tag1_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 6.
t_tag6_to_wrld = np.transpose(np.multiply(np.array([[8, 4, 0.5]]), ft_to_cm))
R_tag6_to_wrld = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0]
])
T_tag6_to_wrld = np.concatenate((np.concatenate((R_tag6_to_wrld, t_tag6_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 17.
t_tag17_to_wrld = np.transpose(np.multiply(np.array([[6.8284, 6.8284, 0.5]]), ft_to_cm))
R_tag17_to_wrld = np.array([
    [ 0.7071,  0,  0.7071],
    [-0.7071,  0,  0.7071],
    [ 0,      -1,  0     ]
])
T_tag17_to_wrld = np.concatenate((np.concatenate((R_tag17_to_wrld, t_tag17_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 40.
t_tag40_to_wrld = np.transpose(np.multiply(np.array([[4, 8, 0.5]]), ft_to_cm))
R_tag40_to_wrld = np.array([
    [ 1,  0,  0],
    [ 0,  0,  1],
    [ 0, -1,  0]
])
T_tag40_to_wrld = np.concatenate((np.concatenate((R_tag40_to_wrld, t_tag40_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 45.
t_tag45_to_wrld = np.transpose(np.multiply(np.array([[1.1716, 6.8284, 0.5]]), ft_to_cm))
R_tag45_to_wrld = np.array([
    [ 0.7071,  0, -0.7071],
    [ 0.7071,  0,  0.7071],
    [ 0,      -1,  0     ]
])
T_tag45_to_wrld = np.concatenate((np.concatenate((R_tag45_to_wrld, t_tag45_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 48.
t_tag48_to_wrld = np.transpose(np.multiply(np.array([[0, 4, 0.5]]), ft_to_cm))
R_tag48_to_wrld = np.array([
    [ 0,  0, -1],
    [ 1,  0,  0],
    [ 0, -1,  0]
])
T_tag48_to_wrld = np.concatenate((np.concatenate((R_tag48_to_wrld, t_tag48_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Calculate transformation matrix for tag 97.
t_tag49_to_wrld = np.transpose(np.multiply(np.array([[1.1716, 1.1716, 0.5]]), ft_to_cm))
R_tag49_to_wrld = np.array([
    [-0.7071,  0, -0.7071],
    [ 0.7071,  0, -0.7071],
    [ 0,      -1,  0     ]
])
T_tag49_to_wrld = np.concatenate((np.concatenate((R_tag49_to_wrld, t_tag49_to_wrld), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

# Save info
np.save("T_tag0_to_wrld.npy", T_tag0_to_wrld)
np.save("T_tag1_to_wrld.npy", T_tag1_to_wrld)
np.save("T_tag6_to_wrld.npy", T_tag6_to_wrld)
np.save("T_tag17_to_wrld.npy", T_tag17_to_wrld)
np.save("T_tag40_to_wrld.npy", T_tag40_to_wrld)
np.save("T_tag45_to_wrld.npy", T_tag45_to_wrld)
np.save("T_tag48_to_wrld.npy", T_tag48_to_wrld)
np.save("T_tag49_to_wrld.npy", T_tag49_to_wrld)