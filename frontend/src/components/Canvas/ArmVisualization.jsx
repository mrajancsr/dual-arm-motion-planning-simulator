/**
 * Arm visualization component
 */

import { getAllJointPositions } from '../../utils/kinematics';

export function drawArms(canvas, armType, config, armParams, leftBase, rightBase) {
  if (!canvas || !config) return;
  
  const numJoints = armType === '2-link' ? 2 : armType === '3-link' ? 3 : 6;
  const leftConfig = config.slice(0, numJoints);
  const rightConfig = config.slice(numJoints);
  
  // Get joint positions
  const leftPositions = getAllJointPositions(leftConfig, armParams, armType, leftBase);
  const rightPositions = getAllJointPositions(rightConfig, armParams, armType, rightBase);
  
  // Draw left arm
  for (let i = 0; i < leftPositions.length - 1; i++) {
    canvas.drawLine(leftPositions[i], leftPositions[i + 1], '#3b82f6', 4);
  }
  
  // Draw right arm
  for (let i = 0; i < rightPositions.length - 1; i++) {
    canvas.drawLine(rightPositions[i], rightPositions[i + 1], '#ef4444', 4);
  }
  
  // Draw joints
  leftPositions.forEach((pos, i) => {
    canvas.drawCircle(pos, i === 0 ? 0.1 : 0.08, '#3b82f6', true);
  });
  
  rightPositions.forEach((pos, i) => {
    canvas.drawCircle(pos, i === 0 ? 0.1 : 0.08, '#ef4444', true);
  });
  
  // Draw end-effectors (larger)
  canvas.drawCircle(
    leftPositions[leftPositions.length - 1],
    0.12,
    '#60a5fa',
    true
  );
  canvas.drawCircle(
    rightPositions[rightPositions.length - 1],
    0.12,
    '#f87171',
    true
  );
}

