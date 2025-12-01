/**
 * Horizontal navigation bar for demo configurations
 */

import React from 'react';

export function DemoNavBar({ demos, activeDemoId, onSelectDemo, isPlanning }) {
  if (!demos || demos.length === 0) {
    return null;
  }

  const getDifficultyColor = (difficulty) => {
    switch (difficulty) {
      case 'simple':
        return 'bg-green-600 hover:bg-green-700';
      case 'medium':
        return 'bg-yellow-600 hover:bg-yellow-700';
      case 'complex':
        return 'bg-red-600 hover:bg-red-700';
      default:
        return 'bg-blue-600 hover:bg-blue-700';
    }
  };

  const getDifficultyBadge = (difficulty) => {
    switch (difficulty) {
      case 'simple':
        return 'bg-green-500/20 text-green-300 border-green-500/30';
      case 'medium':
        return 'bg-yellow-500/20 text-yellow-300 border-yellow-500/30';
      case 'complex':
        return 'bg-red-500/20 text-red-300 border-red-500/30';
      default:
        return 'bg-blue-500/20 text-blue-300 border-blue-500/30';
    }
  };

  return (
    <div className="bg-gradient-to-r from-gray-900 via-gray-800 to-gray-900 border-b border-gray-700 shadow-lg">
      <div className="px-6 py-4">
        <div className="flex items-center justify-between mb-3">
          <h2 className="text-lg font-bold text-white flex items-center gap-2">
            <span className="text-2xl">📚</span>
            Example Scenarios
          </h2>
          <span className="text-xs text-gray-400">
            Click to load a demo configuration
          </span>
        </div>
        
        <div className="flex gap-3 overflow-x-auto pb-2">
          {demos.map((demo) => {
            const isActive = demo.id === activeDemoId;
            const difficulty = demo.metadata?.difficulty || 'medium';
            
            return (
              <button
                key={demo.id}
                onClick={() => onSelectDemo(demo.id)}
                disabled={isPlanning}
                className={`
                  flex-shrink-0 px-4 py-3 rounded-lg border-2 transition-all
                  ${isActive 
                    ? 'border-blue-500 bg-blue-600/30 shadow-lg shadow-blue-500/20' 
                    : 'border-gray-700 bg-gray-800/50 hover:bg-gray-700/70'
                  }
                  ${isPlanning ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}
                  group relative
                `}
              >
                <div className="flex flex-col items-start gap-2 min-w-[200px]">
                  <div className="flex items-center justify-between w-full">
                    <span className={`
                      text-sm font-semibold
                      ${isActive ? 'text-white' : 'text-gray-300 group-hover:text-white'}
                    `}>
                      {demo.metadata?.name || demo.id}
                    </span>
                    
                    <span className={`
                      text-xs px-2 py-0.5 rounded-full border font-medium uppercase tracking-wide
                      ${getDifficultyBadge(difficulty)}
                    `}>
                      {difficulty}
                    </span>
                  </div>
                  
                  {demo.metadata?.description && (
                    <p className={`
                      text-xs line-clamp-2
                      ${isActive ? 'text-gray-200' : 'text-gray-400 group-hover:text-gray-300'}
                    `}>
                      {demo.metadata.description}
                    </p>
                  )}
                </div>
                
                {isActive && (
                  <div className="absolute -bottom-1 left-1/2 transform -translate-x-1/2 w-3 h-3 bg-blue-500 rotate-45 border-b-2 border-r-2 border-gray-700"></div>
                )}
              </button>
            );
          })}
        </div>
      </div>
    </div>
  );
}


