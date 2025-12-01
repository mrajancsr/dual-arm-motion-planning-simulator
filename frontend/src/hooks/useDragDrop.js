/**
 * Custom hook for drag and drop interactions
 */

import { useState, useCallback } from 'react';
import { screenToWorld } from '../utils/geometry';

export function useDragDrop(canvasRef, worldBounds) {
  const [dragging, setDragging] = useState(null);
  
  const startDrag = useCallback((item, event) => {
    setDragging(item);
  }, []);
  
  const handleMouseMove = useCallback((event) => {
    if (!dragging || !canvasRef.current) return null;
    
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const screenX = event.clientX - rect.left;
    const screenY = event.clientY - rect.top;
    
    const worldPos = screenToWorld(
      [screenX, screenY],
      canvas.width,
      canvas.height,
      worldBounds
    );
    
    return {
      item: dragging,
      worldPos
    };
  }, [dragging, canvasRef, worldBounds]);
  
  const endDrag = useCallback(() => {
    setDragging(null);
  }, []);
  
  return {
    dragging,
    startDrag,
    handleMouseMove,
    endDrag
  };
}

