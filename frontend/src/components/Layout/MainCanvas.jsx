/**
 * Main canvas area for visualization
 */

export function MainCanvas({ children }) {
  return (
    <div className="flex-1 relative bg-dark-bg">
      {children}
    </div>
  );
}

