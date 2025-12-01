/**
 * Forward kinematics utilities for visualization
 */

/**
 * Compute forward kinematics for an arm
 * @param {number[]} joints - Joint angles
 * @param {object} params - Arm parameters (L1, L2, etc.)
 * @param {string} armType - Arm type ('2-link', '3-link', '6-link')
 * @returns {number[]} End-effector position [x, y]
 */
export function forwardKinematics(joints, params, armType) {
  if (armType === '2-link') {
    const { L1, L2 } = params;
    const [theta1, theta2] = joints;
    
    const x = L1 * Math.cos(theta1) + L2 * Math.cos(theta1 + theta2);
    const y = L1 * Math.sin(theta1) + L2 * Math.sin(theta1 + theta2);
    
    return [x, y];
  }
  
  if (armType === '3-link') {
    const { L1, L2, L3 } = params;
    const [theta1, theta2, theta3] = joints;
    
    const a1 = theta1;
    const a2 = theta1 + theta2;
    const a3 = theta1 + theta2 + theta3;
    
    const x = L1 * Math.cos(a1) + L2 * Math.cos(a2) + L3 * Math.cos(a3);
    const y = L1 * Math.sin(a1) + L2 * Math.sin(a2) + L3 * Math.sin(a3);
    
    return [x, y];
  }
  
  if (armType === '6-link') {
    const lengths = [params.L1, params.L2, params.L3, params.L4, params.L5, params.L6];
    let cumulativeAngle = 0;
    let x = 0, y = 0;
    
    for (let i = 0; i < 6; i++) {
      cumulativeAngle += joints[i];
      x += lengths[i] * Math.cos(cumulativeAngle);
      y += lengths[i] * Math.sin(cumulativeAngle);
    }
    
    return [x, y];
  }
  
  return [0, 0];
}

/**
 * Compute all joint positions for an arm (for drawing links)
 * @param {number[]} joints - Joint angles
 * @param {object} params - Arm parameters
 * @param {string} armType - Arm type
 * @param {number[]} base - Base position [x, y]
 * @returns {number[][]} Array of positions [[x0, y0], [x1, y1], ...]
 */
export function getAllJointPositions(joints, params, armType, base = [0, 0]) {
  const positions = [[...base]];
  
  if (armType === '2-link') {
    const { L1, L2 } = params;
    const [theta1, theta2] = joints;
    
    const joint1 = [
      base[0] + L1 * Math.cos(theta1),
      base[1] + L1 * Math.sin(theta1)
    ];
    
    const endEff = [
      joint1[0] + L2 * Math.cos(theta1 + theta2),
      joint1[1] + L2 * Math.sin(theta1 + theta2)
    ];
    
    positions.push(joint1, endEff);
  }
  else if (armType === '3-link') {
    const { L1, L2, L3 } = params;
    const [theta1, theta2, theta3] = joints;
    
    const a1 = theta1;
    const a2 = theta1 + theta2;
    const a3 = theta1 + theta2 + theta3;
    
    const joint1 = [
      base[0] + L1 * Math.cos(a1),
      base[1] + L1 * Math.sin(a1)
    ];
    
    const joint2 = [
      joint1[0] + L2 * Math.cos(a2),
      joint1[1] + L2 * Math.sin(a2)
    ];
    
    const endEff = [
      joint2[0] + L3 * Math.cos(a3),
      joint2[1] + L3 * Math.sin(a3)
    ];
    
    positions.push(joint1, joint2, endEff);
  }
  else if (armType === '6-link') {
    const lengths = [params.L1, params.L2, params.L3, params.L4, params.L5, params.L6];
    let cumulativeAngle = 0;
    let currentPos = [...base];
    
    for (let i = 0; i < 6; i++) {
      cumulativeAngle += joints[i];
      currentPos = [
        currentPos[0] + lengths[i] * Math.cos(cumulativeAngle),
        currentPos[1] + lengths[i] * Math.sin(cumulativeAngle)
      ];
      positions.push([...currentPos]);
    }
  }
  
  return positions;
}

/**
 * Convert configuration to workspace positions
 * @param {number[]} config - Full configuration [θ1_left, θ2_left, ..., θ1_right, ...]
 * @param {string} armType - Arm type
 * @param {object} armParams - Arm parameters
 * @param {number[]} leftBase - Left arm base position
 * @param {number[]} rightBase - Right arm base position
 * @returns {object} {left: [x, y], right: [x, y]}
 */
export function configToWorkspace(config, armType, armParams, leftBase, rightBase) {
  const numJoints = armType === '2-link' ? 2 : armType === '3-link' ? 3 : 6;
  
  const leftConfig = config.slice(0, numJoints);
  const rightConfig = config.slice(numJoints, numJoints * 2);
  
  const leftLocal = forwardKinematics(leftConfig, armParams, armType);
  const rightLocal = forwardKinematics(rightConfig, armParams, armType);
  
  return {
    left: [leftBase[0] + leftLocal[0], leftBase[1] + leftLocal[1]],
    right: [rightBase[0] + rightLocal[0], rightBase[1] + rightLocal[1]]
  };
}

