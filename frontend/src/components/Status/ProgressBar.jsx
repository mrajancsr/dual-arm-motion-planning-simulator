/**
 * Progress bar for planning
 */

export function ProgressBar({ progress }) {
  if (!progress) return null;
  
  const percentage = Math.round((progress.iterations / progress.max_iterations) * 100);
  
  return (
    <div className="space-y-2">
      <div className="flex justify-between text-xs text-gray-400">
        <span>Progress</span>
        <span>{progress.iterations} / {progress.max_iterations} ({percentage}%)</span>
      </div>
      <div className="w-full h-2 bg-gray-700 rounded-full overflow-hidden">
        <div
          className="h-full bg-blue-500 transition-all duration-300"
          style={{ width: `${percentage}%` }}
        />
      </div>
    </div>
  );
}

