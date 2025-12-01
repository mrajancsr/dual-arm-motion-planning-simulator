/**
 * Status indicator component
 */

export function StatusIndicator({ status, error, progress }) {
  const statusConfig = {
    idle: { color: 'bg-gray-500', text: 'Ready', icon: '●' },
    starting: { color: 'bg-yellow-500', text: 'Starting...', icon: '◐' },
    running: { color: 'bg-blue-500', text: 'Planning...', icon: '◐' },
    completed: { color: 'bg-green-500', text: 'Completed', icon: '✓' },
    failed: { color: 'bg-red-500', text: 'Failed', icon: '✗' }
  };
  
  const config = statusConfig[status] || statusConfig.idle;
  
  return (
    <div className="space-y-2">
      <div className={`flex items-center gap-2 p-3 rounded-lg ${
        status === 'running' || status === 'starting' ? 'bg-blue-900/30' :
        status === 'completed' ? 'bg-green-900/30' :
        status === 'failed' ? 'bg-red-900/30' :
        'bg-gray-800'
      }`}>
        <span className={`w-3 h-3 rounded-full ${config.color} ${
          status === 'running' || status === 'starting' ? 'animate-pulse' : ''
        }`} />
        <span className="text-white font-medium">{config.text}</span>
      </div>
      
      {/* Show current phase during planning */}
      {status === 'running' && progress?.current_phase && (
        <div className="p-2 bg-blue-900/30 border border-blue-500/50 rounded text-xs text-blue-200">
          📊 {progress.current_phase}
        </div>
      )}
      
      {error && (
        <div className="p-2 bg-red-900/30 border border-red-700 rounded text-xs text-red-200">
          {error}
        </div>
      )}
    </div>
  );
}

