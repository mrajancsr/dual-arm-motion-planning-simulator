/**
 * Geometry utilities for collision detection and rendering
 */

/**
 * Check if point is inside circle
 */
export function pointInCircle(point, circle) {
  const dx = point[0] - circle.center[0];
  const dy = point[1] - circle.center[1];
  return (dx * dx + dy * dy) <= (circle.radius * circle.radius);
}

/**
 * Get distance from point to circle center
 */
export function distanceToCircle(point, circle) {
  const dx = point[0] - circle.center[0];
  const dy = point[1] - circle.center[1];
  return Math.sqrt(dx * dx + dy * dy) - circle.radius;
}

/**
 * Convert world coordinates to screen coordinates
 */
export function worldToScreen(worldPos, canvasWidth, canvasHeight, worldBounds) {
  const { minX, maxX, minY, maxY } = worldBounds;
  
  const scaleX = canvasWidth / (maxX - minX);
  const scaleY = canvasHeight / (maxY - minY);
  const scale = Math.min(scaleX, scaleY);
  
  const centerX = canvasWidth / 2;
  const centerY = canvasHeight / 2;
  
  const screenX = centerX + (worldPos[0] - (minX + maxX) / 2) * scale;
  const screenY = centerY - (worldPos[1] - (minY + maxY) / 2) * scale;  // Flip Y
  
  return [screenX, screenY, scale];
}

/**
 * Convert screen coordinates to world coordinates
 */
export function screenToWorld(screenPos, canvasWidth, canvasHeight, worldBounds) {
  const { minX, maxX, minY, maxY } = worldBounds;
  
  const scaleX = canvasWidth / (maxX - minX);
  const scaleY = canvasHeight / (maxY - minY);
  const scale = Math.min(scaleX, scaleY);
  
  const centerX = canvasWidth / 2;
  const centerY = canvasHeight / 2;
  
  const worldX = (screenPos[0] - centerX) / scale + (minX + maxX) / 2;
  const worldY = -((screenPos[1] - centerY) / scale) + (minY + maxY) / 2;  // Flip Y
  
  return [worldX, worldY];
}

