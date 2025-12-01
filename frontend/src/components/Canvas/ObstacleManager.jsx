/**
 * Obstacle manager for adding and editing obstacles
 */

export function drawObstacles(canvas, obstacles) {
  if (!canvas || !obstacles) return;
  
  obstacles.forEach((obstacle) => {
    if (obstacle.type === 'circle') {
      canvas.drawCircle(
        obstacle.center,
        obstacle.radius,
        '#f97316',
        false
      );
      
      // Fill with semi-transparent orange
      const ctx = canvas.getContext();
      if (ctx) {
        const [x, y, scale] = canvas.toScreen(obstacle.center);
        ctx.beginPath();
        ctx.arc(x, y, obstacle.radius * scale, 0, 2 * Math.PI);
        ctx.fillStyle = '#f9731633';
        ctx.fill();
      }
    }
  });
}

