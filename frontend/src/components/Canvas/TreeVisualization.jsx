/**
 * RRT* tree visualization component
 */

import { configToWorkspace } from '../../utils/kinematics';

export function drawTree(canvas, tree, armType, armParams, leftBase, rightBase) {
  if (!canvas || !tree || !tree.nodes || tree.nodes.length === 0) return;
  
  const nodes = tree.nodes;
  
  // Draw edges first (so they're behind nodes)
  nodes.forEach((node) => {
    // Validate node has required properties
    if (!node || !node.config) return;
    
    if (node.parent !== null && node.parent !== undefined && node.parent >= 0) {
      const parentNode = nodes[node.parent];
      
      // Validate parent node exists and has config
      if (!parentNode || !parentNode.config) return;
      
      // Convert to workspace positions
      const nodeWS = configToWorkspace(
        node.config,
        armType,
        armParams,
        leftBase,
        rightBase
      );
      const parentWS = configToWorkspace(
        parentNode.config,
        armType,
        armParams,
        leftBase,
        rightBase
      );
      
      // Draw edge between end-effectors (more visible)
      canvas.drawLine(nodeWS.left, parentWS.left, '#3b82f6aa', 1);
      canvas.drawLine(nodeWS.right, parentWS.right, '#ef4444aa', 1);
    }
  });
  
  // Draw nodes
  nodes.forEach((node) => {
    // Validate node has required properties
    if (!node || !node.config) return;
    
    const ws = configToWorkspace(
      node.config,
      armType,
      armParams,
      leftBase,
      rightBase
    );
    
    // Draw small circles at end-effector positions (more visible)
    canvas.drawPoint(ws.left, '#3b82f6cc', 3);
    canvas.drawPoint(ws.right, '#ef4444cc', 3);
  });
}

