/**
 * Start/Stop planning button
 */

export function StartStopButton({ state, onStart, onStop }) {
  const isRunning = state === 'running' || state === 'starting';
  
  return (
    <button
      onClick={isRunning ? onStop : onStart}
      disabled={state === 'starting'}
      className={`w-full py-3 px-4 rounded-lg font-semibold text-white transition-colors ${
        isRunning
          ? 'bg-red-600 hover:bg-red-700'
          : 'bg-green-600 hover:bg-green-700'
      } disabled:opacity-50 disabled:cursor-not-allowed`}
    >
      {state === 'starting' && '⏳ Starting...'}
      {state === 'running' && '⏹ Stop Planning'}
      {(state === 'idle' || state === 'completed' || state === 'failed') && '▶ Start Planning'}
    </button>
  );
}

