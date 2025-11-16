import numpy as np
try:
    from .arms.two_link_arm import TwoLinkArm
except ImportError:
    from arms.two_link_arm import TwoLinkArm

arm = TwoLinkArm(L1=1.0, L2=0.7)
targets = [(0.3, 0.3), (0.4, 0.1), (-0.25, 0.4)]
results = []

for (x, y) in targets:
    thetas = arm.ik_geometric(x, y)
    if thetas is not None:
        fk = arm.forward_kinematics(*thetas[0])  # elbow-up, for instance
        error = np.linalg.norm(np.array(fk) - np.array([x, y]))
        results.append((x, y, round(error, 6)))
    else:
        results.append((x, y, "No solution"))

print(results)