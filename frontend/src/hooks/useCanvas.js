/**
 * Custom hook for canvas management and drawing utilities
 */

import { useRef, useEffect, useState } from 'react';
import { worldToScreen } from '../utils/geometry';

export function useCanvas(worldBounds = { minX: -4, maxX: 4, minY: -2, maxY: 4 }) {
  const canvasRef = useRef(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 });
  
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    // Handle resize
    const handleResize = () => {
      const parent = canvas.parentElement;
      if (parent) {
        const width = parent.clientWidth;
        const height = parent.clientHeight;
        // Only update if dimensions actually changed
        setDimensions(prev => {
          if (prev.width !== width || prev.height !== height) {
            return { width, height };
          }
          return prev;
        });
      }
    };
    
    // Initial resize
    handleResize();
    
    // Use ResizeObserver to detect parent size changes
    const resizeObserver = new ResizeObserver(() => {
      handleResize();
    });
    
    if (canvas.parentElement) {
      resizeObserver.observe(canvas.parentElement);
    }
    
    // Also listen for window resize as backup
    window.addEventListener('resize', handleResize);
    
    return () => {
      resizeObserver.disconnect();
      window.removeEventListener('resize', handleResize);
    };
  }, []);
  
  const getContext = () => {
    const canvas = canvasRef.current;
    return canvas?.getContext('2d');
  };
  
  const clear = () => {
    const ctx = getContext();
    if (!ctx) return;
    
    ctx.clearRect(0, 0, dimensions.width, dimensions.height);
  };
  
  const toScreen = (worldPos) => {
    return worldToScreen(worldPos, dimensions.width, dimensions.height, worldBounds);
  };
  
  const drawLine = (from, to, color = '#ffffff', width = 2) => {
    const ctx = getContext();
    if (!ctx) return;
    
    const [x1, y1, scale] = toScreen(from);
    const [x2, y2] = toScreen(to);
    
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
  };
  
  const drawCircle = (center, radius, color = '#ffffff', fill = false) => {
    const ctx = getContext();
    if (!ctx) return;
    
    const [x, y, scale] = toScreen(center);
    const r = radius * scale;
    
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    
    if (fill) {
      ctx.fillStyle = color;
      ctx.fill();
    } else {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  };
  
  const drawPoint = (pos, color = '#ffffff', size = 5) => {
    const ctx = getContext();
    if (!ctx) return;
    
    const [x, y] = toScreen(pos);
    
    ctx.beginPath();
    ctx.arc(x, y, size, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
  };
  
  const drawText = (text, pos, color = '#ffffff', fontSize = 12) => {
    const ctx = getContext();
    if (!ctx) return;
    
    const [x, y] = toScreen(pos);
    
    ctx.fillStyle = color;
    ctx.font = `${fontSize}px sans-serif`;
    ctx.fillText(text, x, y);
  };
  
  return {
    canvasRef,
    dimensions,
    clear,
    toScreen,
    drawLine,
    drawCircle,
    drawPoint,
    drawText,
    getContext
  };
}

