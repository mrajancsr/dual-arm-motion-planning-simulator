/**
 * Optimization feature toggles
 */

export function OptimizationToggles({ config, onChange, disabled }) {
  const updateConfig = (key, value) => {
    onChange({ ...config, [key]: value });
  };
  
  const Toggle = ({ label, checked, onChange, disabled }) => (
    <label className="flex items-center justify-between p-2 rounded hover:bg-gray-700 cursor-pointer">
      <span className="text-sm text-gray-300">{label}</span>
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        disabled={disabled}
        className="w-5 h-5 accent-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
      />
    </label>
  );
  
  return (
    <div className="space-y-2">
      <h3 className="text-lg font-semibold text-white">Optimizations</h3>
      
      <div className="space-y-1 bg-gray-800 rounded-lg p-2">
        <Toggle
          label="KD-Tree Search"
          checked={config.use_kdtree}
          onChange={(val) => updateConfig('use_kdtree', val)}
          disabled={disabled}
        />
        
        <Toggle
          label="Adaptive Step Size"
          checked={config.use_adaptive_step}
          onChange={(val) => updateConfig('use_adaptive_step', val)}
          disabled={disabled}
        />
        
        <div className="space-y-2 p-2">
          <label className="flex justify-between text-sm text-gray-300">
            <span>Workspace Weight</span>
            <span className="text-green-400">{config.workspace_weight.toFixed(2)}</span>
          </label>
          <input
            type="range"
            min="0"
            max="1"
            step="0.1"
            value={config.workspace_weight}
            onChange={(e) => updateConfig('workspace_weight', parseFloat(e.target.value))}
            disabled={disabled}
            className="w-full accent-green-500 disabled:opacity-50"
          />
        </div>
      </div>
    </div>
  );
}

