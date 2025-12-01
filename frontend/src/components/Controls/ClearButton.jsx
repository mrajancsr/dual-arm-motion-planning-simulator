/**
 * Clear button to reset all configuration and visuals
 */

export function ClearButton({ onClear, disabled }) {
  return (
    <button
      onClick={onClear}
      disabled={disabled}
      className="w-full py-2 px-4 rounded-lg font-semibold text-white transition-colors bg-gray-600 hover:bg-gray-700 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
    >
      <span>🗑️</span>
      <span>Clear All</span>
    </button>
  );
}

