/**
 * Metrics display for planning statistics
 */

export function MetricsDisplay({ progress, path, planningTime, status }) {
  if (!progress) return null;
  
  const getProgressPercentage = () => {
    if (!progress.iterations || !progress.max_iterations || progress.max_iterations === 0) return 0;
    return Math.min(100, (progress.iterations / progress.max_iterations) * 100);
  };
  
  // Calculate time remaining (ETA)
  const getTimeRemaining = () => {
    if (!progress.iterations || progress.iterations === 0 || status !== 'running') return null;
    if (!progress.start_time) return null;
    
    const elapsed = Date.now() / 1000 - progress.start_time;
    const iterationsPerSecond = progress.iterations / elapsed;
    const remainingIterations = progress.max_iterations - progress.iterations;
    const estimatedSecondsRemaining = remainingIterations / iterationsPerSecond;
    
    return estimatedSecondsRemaining;
  };
  
  const formatTimeRemaining = (seconds) => {
    if (!seconds || seconds < 0) return '';
    if (seconds < 60) return `~${Math.ceil(seconds)}s`;
    const minutes = Math.floor(seconds / 60);
    const secs = Math.ceil(seconds % 60);
    return `~${minutes}m ${secs}s`;
  };
  
  const timeRemaining = getTimeRemaining();
  
  return (
    <div className="space-y-2">
      <h3 className="text-sm font-semibold text-gray-300">Planning Progress</h3>
      
      {/* Progress bar with ETA */}
      {progress.max_iterations > 0 && (
        <div>
          <div className="flex justify-between text-xs text-gray-400 mb-1">
            <span>Iterations</span>
            <span>{progress.iterations || 0} / {progress.max_iterations}</span>
          </div>
          <div className="w-full bg-gray-700 rounded-full h-3 overflow-hidden">
            <div 
              className="bg-gradient-to-r from-blue-500 to-purple-500 h-3 rounded-full transition-all duration-200 ease-out"
              style={{ width: `${getProgressPercentage()}%` }}
            />
          </div>
          {timeRemaining && timeRemaining > 0.5 && (
            <div className="text-xs text-gray-400 mt-1 text-right">
              ETA: {formatTimeRemaining(timeRemaining)}
            </div>
          )}
        </div>
      )}
      
      <div className="grid grid-cols-2 gap-2 text-xs">
        <div className="bg-gray-800 p-2 rounded">
          <div className="text-gray-400">Tree Size</div>
          <div className="text-white font-semibold">{progress.tree_size || 0}</div>
        </div>
        
        <div className="bg-gray-800 p-2 rounded">
          <div className="text-gray-400">Iterations</div>
          <div className="text-white font-semibold">{progress.iterations || 0}</div>
        </div>
        
        {progress.goal_found && (
          <>
            <div className="bg-gray-800 p-2 rounded col-span-2">
              <div className="text-green-400 font-semibold flex items-center gap-1">
                <span>✓</span>
                <span>Goal Reached!</span>
              </div>
            </div>
            
            {path && (
              <div className="bg-gray-800 p-2 rounded">
                <div className="text-gray-400">Path Length</div>
                <div className="text-green-400 font-semibold">
                  {path.length || 0} nodes
                </div>
              </div>
            )}
            
            {planningTime && (
              <div className="bg-gray-800 p-2 rounded">
                <div className="text-gray-400">Time</div>
                <div className="text-green-400 font-semibold">
                  {planningTime.toFixed(2)}s
                </div>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
}

