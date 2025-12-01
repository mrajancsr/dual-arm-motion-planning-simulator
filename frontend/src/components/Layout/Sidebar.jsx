/**
 * Sidebar component for controls and configuration
 */

export function Sidebar({ children }) {
  return (
    <div className="w-80 bg-dark-panel border-r border-gray-700 overflow-y-auto flex flex-col">
      <div className="p-4 border-b border-gray-700">
        <h1 className="text-2xl font-bold text-white">
          🦾 Dual-Arm Planner
        </h1>
        <p className="text-sm text-gray-400 mt-1">
          Interactive Motion Planning
        </p>
      </div>
      
      <div className="flex-1 p-4 space-y-6">
        {children}
      </div>
    </div>
  );
}

