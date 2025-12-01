/**
 * Path visualization component with animation
 */

export function drawPath(canvas, path) {
  if (!canvas || !path || !path.path || path.path.length === 0) return;
  
  // Draw path as thick green line
  for (let i = 0; i < path.path.length - 1; i++) {
    const current = path.path[i];
    const next = path.path[i + 1];
    
    canvas.drawLine(current.left_ee, next.left_ee, '#22c55e', 3);
    canvas.drawLine(current.right_ee, next.right_ee, '#22c55e', 3);
  }
  
  // Draw waypoints
  path.path.forEach((waypoint) => {
    canvas.drawPoint(waypoint.left_ee, '#22c55e', 4);
    canvas.drawPoint(waypoint.right_ee, '#22c55e', 4);
  });
}

