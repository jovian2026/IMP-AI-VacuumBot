import numpy as np
import gtsam
import matplotlib.pyplot as plt

graph = gtsam.NonlinearFactorGraph()
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
odom_noise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

graph.add(gtsam.PriorFactorPose2(1, gtsam.Pose2(0, 0, 0), prior_noise))

# Odometry: drive 2 m, turn 90 deg, repeat (a square)
moves = [gtsam.Pose2(2, 0, 0)] + [gtsam.Pose2(2, 0, np.pi / 2)] * 3
for i, m in enumerate(moves):
    graph.add(gtsam.BetweenFactorPose2(i + 1, i + 2, m, odom_noise))

# LOOP CLOSURE: pose 5 sees the same place as pose 2
graph.add(gtsam.BetweenFactorPose2(5, 2, gtsam.Pose2(2, 0, np.pi / 2), odom_noise))

# Drifted starting guess (like dead-reckoning with wheel slip)
guess = [(0.5, 0.0, 0.2), (2.3, 0.1, -0.2), (4.1, 0.1, np.pi/2 + 0.1),
         (4.0, 2.0, np.pi - 0.1), (2.1, 2.1, -np.pi/2 + 0.1)]
initial = gtsam.Values()
for i, (x, y, th) in enumerate(guess):
    initial.insert(i + 1, gtsam.Pose2(x, y, th))

result = gtsam.LevenbergMarquardtOptimizer(
    graph, initial, gtsam.LevenbergMarquardtParams()).optimize()

def xy(vals):
    return [(vals.atPose2(k).x(), vals.atPose2(k).y()) for k in range(1, 6)]

before, after = np.array(xy(initial)), np.array(xy(result))
plt.plot(*before.T, 'r.--', label='drifted guess')
plt.plot(*after.T, 'g.-', label='after loop closure')
plt.axis('equal'); plt.legend(); plt.show()