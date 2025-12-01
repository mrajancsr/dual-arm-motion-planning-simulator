/**
 * Demo controls for managing saved/fresh results
 */

import React from 'react';

export function DemoControls({ 
  activeDemoId, 
  resultMode, 
  onResetToSaved, 
  onRerunFresh,
  onSaveAsDemo,
  isPlanning,
  hasResults 
}) {
  if (!activeDemoId) {
    // Show only save button when no demo is active
    return (
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300">Demo Actions</h3>
        <button
          onClick={onSaveAsDemo}
          disabled={isPlanning || !hasResults}
          className={`
            w-full px-4 py-2 rounded-lg border-2 text-sm font-medium
            transition-all flex items-center justify-center gap-2
            ${isPlanning || !hasResults
              ? 'bg-gray-700 border-gray-600 text-gray-500 cursor-not-allowed'
              : 'bg-purple-600 border-purple-500 text-white hover:bg-purple-700 hover:shadow-lg'
            }
          `}
        >
          <span>💾</span>
          Save as Demo
        </button>
        {!hasResults && (
          <p className="text-xs text-gray-500 text-center">
            Run planning first to save a demo
          </p>
        )}
      </div>
    );
  }

  return (
    <div className="space-y-3">
      <div className="flex items-center justify-between">
        <h3 className="text-sm font-semibold text-gray-300">Demo Mode</h3>
        <span className={`
          text-xs px-2 py-1 rounded font-medium
          ${resultMode === 'saved' 
            ? 'bg-blue-600/30 text-blue-300 border border-blue-500/30' 
            : 'bg-green-600/30 text-green-300 border border-green-500/30'
          }
        `}>
          {resultMode === 'saved' ? '📁 Saved Results' : '🔄 Fresh Run'}
        </span>
      </div>

      <div className="space-y-2">
        <button
          onClick={onResetToSaved}
          disabled={isPlanning || resultMode === 'saved'}
          className={`
            w-full px-4 py-2 rounded-lg border-2 text-sm font-medium
            transition-all flex items-center justify-center gap-2
            ${isPlanning || resultMode === 'saved'
              ? 'bg-gray-700 border-gray-600 text-gray-500 cursor-not-allowed'
              : 'bg-blue-600 border-blue-500 text-white hover:bg-blue-700 hover:shadow-lg'
            }
          `}
        >
          <span>↩️</span>
          Reset to Saved
        </button>

        <button
          onClick={onRerunFresh}
          disabled={isPlanning}
          className={`
            w-full px-4 py-2 rounded-lg border-2 text-sm font-medium
            transition-all flex items-center justify-center gap-2
            ${isPlanning
              ? 'bg-gray-700 border-gray-600 text-gray-500 cursor-not-allowed'
              : 'bg-green-600 border-green-500 text-white hover:bg-green-700 hover:shadow-lg'
            }
          `}
        >
          <span>🔄</span>
          {isPlanning ? 'Planning...' : 'Rerun Fresh'}
        </button>

        <button
          onClick={onSaveAsDemo}
          disabled={isPlanning || !hasResults}
          className={`
            w-full px-4 py-2 rounded-lg border-2 text-sm font-medium
            transition-all flex items-center justify-center gap-2
            ${isPlanning || !hasResults
              ? 'bg-gray-700 border-gray-600 text-gray-500 cursor-not-allowed'
              : 'bg-purple-600 border-purple-500 text-white hover:bg-purple-700 hover:shadow-lg'
            }
          `}
        >
          <span>💾</span>
          Save as New Demo
        </button>
      </div>

      <div className="pt-2 border-t border-gray-700">
        <p className="text-xs text-gray-400">
          {resultMode === 'saved' 
            ? 'Viewing pre-computed results. Click "Rerun Fresh" to compute new results.'
            : 'Viewing fresh results. Original saved results preserved.'
          }
        </p>
      </div>
    </div>
  );
}


