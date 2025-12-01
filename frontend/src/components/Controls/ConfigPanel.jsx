/**
 * Configuration panel for planning parameters
 */

export function ConfigPanel({ config, onChange, disabled }) {
  const updateConfig = (key, value) => {
    onChange({ ...config, [key]: value });
  };
  
  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold text-white">Planning Parameters</h3>
      
      {/* Max Iterations */}
      <div className="space-y-2">
        <label className="flex justify-between text-sm text-gray-300">
          <span>Max Iterations</span>
          <span className="text-blue-400">{config.max_iterations}</span>
        </label>
        <input
          type="range"
          min="1000"
          max="15000"
          step="1000"
          value={config.max_iterations}
          onChange={(e) => updateConfig('max_iterations', parseInt(e.target.value))}
          disabled={disabled}
          className="w-full accent-blue-500 disabled:opacity-50"
        />
      </div>
      
      {/* Step Size */}
      <div className="space-y-2">
        <label className="flex justify-between text-sm text-gray-300">
          <span>Step Size</span>
          <span className="text-blue-400">{config.step_size.toFixed(2)}</span>
        </label>
        <input
          type="range"
          min="0.05"
          max="0.3"
          step="0.01"
          value={config.step_size}
          onChange={(e) => updateConfig('step_size', parseFloat(e.target.value))}
          disabled={disabled}
          className="w-full accent-blue-500 disabled:opacity-50"
        />
      </div>
      
      {/* Goal Threshold */}
      <div className="space-y-2">
        <label className="flex justify-between text-sm text-gray-300">
          <span>Goal Threshold</span>
          <span className="text-blue-400">{config.goal_threshold.toFixed(2)}</span>
        </label>
        <input
          type="range"
          min="0.05"
          max="0.5"
          step="0.05"
          value={config.goal_threshold}
          onChange={(e) => updateConfig('goal_threshold', parseFloat(e.target.value))}
          disabled={disabled}
          className="w-full accent-blue-500 disabled:opacity-50"
        />
      </div>
    </div>
  );
}

