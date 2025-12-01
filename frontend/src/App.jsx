/**
 * Main application component
 */

import { useState, useEffect, useRef } from 'react';
import { Sidebar } from './components/Layout/Sidebar';
import { MainCanvas } from './components/Layout/MainCanvas';
import { ArmSelector } from './components/Controls/ArmSelector';
import { ConfigPanel } from './components/Controls/ConfigPanel';
import { OptimizationToggles } from './components/Controls/OptimizationToggles';
import { StartStopButton } from './components/Controls/StartStopButton';
import { ClearButton } from './components/Controls/ClearButton';
import { MetricsDisplay } from './components/Status/MetricsDisplay';
import { StatusIndicator } from './components/Status/StatusIndicator';
import { DemoNavBar } from './components/Demos/DemoNavBar';
import { DemoControls } from './components/Demos/DemoControls';
import { drawArms } from './components/Canvas/ArmVisualization';
import { drawTree } from './components/Canvas/TreeVisualization';
import { drawPath } from './components/Canvas/PathVisualization';
import { drawObstacles } from './components/Canvas/ObstacleManager';
import { usePlanner } from './hooks/usePlanner';
import { useDemo } from './hooks/useDemo';
import { useCanvas } from './hooks/useCanvas';
import { configToWorkspace } from './utils/kinematics';
import { screenToWorld } from './utils/geometry';
import { api } from './utils/api';
import './index.css';

// Default configurations
const DEFAULT_ARM_PARAMS = {
  '2-link': { L1: 1.0, L2: 0.7 },
  '3-link': { L1: 1.0, L2: 0.8, L3: 0.6 },
  '6-link': { L1: 0.5, L2: 0.4, L3: 0.3, L4: 0.25, L5: 0.2, L6: 0.15 }
};

// Item positions (what the arms will manipulate)
const DEFAULT_ITEM_START = [0, 1.5]; // Item start position
const DEFAULT_ITEM_GOAL = [0, 2.8];  // Item goal position
const ITEM_GRIP_OFFSET = 0.2; // How far apart arms grip the item

// Default arm bases
const DEFAULT_LEFT_BASE_X = -1.0;
const DEFAULT_RIGHT_BASE_X = 1.0;

// Helper: Get initial config with arms pointing north (fully extended)
const getInitialConfig = (armType, armParams, leftBaseX, rightBaseX) => {
  const numJoints = armType === '2-link' ? 2 : armType === '3-link' ? 3 : 6;
  // All joints at 0 means pointing straight up (north)
  return Array(numJoints * 2).fill(0);
};

const DEFAULT_CONFIG = {
  arm_type: '2-link',
  start: getInitialConfig('2-link', DEFAULT_ARM_PARAMS['2-link'], DEFAULT_LEFT_BASE_X, DEFAULT_RIGHT_BASE_X),
  goal: getInitialConfig('2-link', DEFAULT_ARM_PARAMS['2-link'], DEFAULT_LEFT_BASE_X, DEFAULT_RIGHT_BASE_X),
  obstacles: [
    { type: 'circle', center: [0, 1.0], radius: 0.3 }
  ],
  separation: 2.0,
  max_iterations: 5000,
  step_size: 0.15,
  goal_threshold: 0.2,
  use_kdtree: true,
  workspace_weight: 0.3,
  use_adaptive_step: true,
  arm_params: DEFAULT_ARM_PARAMS['2-link']
};

function App() {
  const [armType, setArmType] = useState('2-link');
  const [config, setConfig] = useState(DEFAULT_CONFIG);
  const [dragging, setDragging] = useState(null); // 'item-start' | 'item-goal' | 'base-left' | 'base-right'
  const [itemStartPos, setItemStartPos] = useState(DEFAULT_ITEM_START);
  const [itemGoalPos, setItemGoalPos] = useState(DEFAULT_ITEM_GOAL);
  const [showTree, setShowTree] = useState(true); // Enable tree viz by default
  
  // Draggable base positions
  const [leftBaseX, setLeftBaseX] = useState(DEFAULT_LEFT_BASE_X);
  const [rightBaseX, setRightBaseX] = useState(DEFAULT_RIGHT_BASE_X);
  
  const { 
    status, 
    progress, 
    tree, 
    path, 
    error, 
    strategy,
    handoffPoint,
    phases,
    startPlanning, 
    stopPlanning,
    clearAll
  } = usePlanner();
  
  const {
    demos,
    activeDemoId,
    activeDemoData,
    resultMode,
    savedResults,
    loadDemo,
    resetToSaved,
    switchToFresh,
    saveAsDemo,
    clearDemo
  } = useDemo();
  
  const canvas = useCanvas({ minX: -4, maxX: 4, minY: -2, maxY: 4 });
  
  // Calculate base positions
  const leftBase = [leftBaseX, 0];
  const rightBase = [rightBaseX, 0];
  
  // Handle arm type change
  const handleArmTypeChange = (newType) => {
    setArmType(newType);
    
    // Update config with new dimensions
    const numJoints = newType === '2-link' ? 2 : newType === '3-link' ? 3 : 6;
    const newStart = Array(numJoints * 2).fill(0).map(() => 0.5);
    const newGoal = Array(numJoints * 2).fill(0).map(() => -0.5);
    
    setConfig({
      ...config,
      arm_type: newType,
      start: newStart,
      goal: newGoal,
      arm_params: DEFAULT_ARM_PARAMS[newType]
    });
  };
  
  // Handle start planning
  const handleStartPlanning = async () => {
    console.log('Starting handoff planning');
    console.log('Item start:', itemStartPos);
    console.log('Item goal:', itemGoalPos);
    console.log('Left base X:', leftBaseX);
    console.log('Right base X:', rightBaseX);
    
    try {
      // Use new handoff planning API
      startPlanning({
        arm_type: armType,
        item_start: itemStartPos,
        item_goal: itemGoalPos,
        left_base_x: leftBaseX,
        right_base_x: rightBaseX,
        obstacles: config.obstacles,
        arm_params: config.arm_params,
        rrt_params: {
          max_iterations: config.max_iterations,
          step_size: config.step_size,
          goal_threshold: config.goal_threshold,
          use_kdtree: config.use_kdtree,
          workspace_weight: config.workspace_weight,
          use_adaptive_step: config.use_adaptive_step
        }
      });
    } catch (error) {
      console.error('Planning error:', error);
      alert(`Error starting planning: ${error.message}`);
    }
  };
  
  // Handle clear all
  const handleClear = () => {
    // Stop any ongoing planning
    stopPlanning();
    
    // Clear planner state (tree, path, strategy, etc.)
    clearAll();
    
    // Clear demo state
    clearDemo();
    
    // Reset configuration to defaults
    setConfig({
      ...DEFAULT_CONFIG,
      arm_type: armType,
      arm_params: DEFAULT_ARM_PARAMS[armType],
      obstacles: [
        { type: 'circle', center: [0, 1.0], radius: 0.3 }
      ]
    });
    
    // Reset item positions
    setItemStartPos(DEFAULT_ITEM_START);
    setItemGoalPos(DEFAULT_ITEM_GOAL);
    
    // Reset base positions
    setLeftBaseX(DEFAULT_LEFT_BASE_X);
    setRightBaseX(DEFAULT_RIGHT_BASE_X);
    
    // Clear dragging state
    setDragging(null);
    
    console.log('Cleared all configuration and visuals');
  };
  
  // Handle demo selection
  const handleSelectDemo = async (demoId) => {
    console.log('Loading demo:', demoId);
    
    const demoData = await loadDemo(demoId);
    if (!demoData) return;
    
    // Load demo configuration
    const demoConfig = demoData.config;
    
    // Update arm type and params
    setArmType(demoConfig.arm_type);
    setConfig(prev => ({
      ...prev,
      arm_type: demoConfig.arm_type,
      arm_params: demoConfig.arm_params,
      obstacles: demoConfig.obstacles || [],
      ...demoConfig.rrt_params
    }));
    
    // Update positions
    setItemStartPos(demoConfig.item_start);
    setItemGoalPos(demoConfig.item_goal);
    setLeftBaseX(demoConfig.left_base_x);
    setRightBaseX(demoConfig.right_base_x);
    
    // Clear any ongoing planning
    stopPlanning();
    clearAll();
    
    console.log('Demo loaded successfully');
  };
  
  // Handle reset to saved results
  const handleResetToSaved = () => {
    resetToSaved();
    console.log('Reset to saved results');
  };
  
  // Handle rerun fresh
  const handleRerunFresh = () => {
    switchToFresh();
    handleStartPlanning();
  };
  
  // Handle save as demo
  const handleSaveAsDemo = async () => {
    const currentState = {
      armType,
      armParams: config.arm_params,
      itemStart: itemStartPos,
      itemGoal: itemGoalPos,
      leftBaseX,
      rightBaseX,
      obstacles: config.obstacles,
      rrtParams: {
        max_iterations: config.max_iterations,
        step_size: config.step_size,
        goal_threshold: config.goal_threshold,
        use_kdtree: config.use_kdtree,
        workspace_weight: config.workspace_weight,
        use_adaptive_step: config.use_adaptive_step
      },
      results: {
        strategy,
        handoffPoint,
        phases,
        path,
        tree,
        progress
      }
    };
    
    await saveAsDemo(currentState);
  };
  
  // Compute IK for a single arm to reach a target
  const computeArmIK = (target, base) => {
    const numJoints = armType === '2-link' ? 2 : armType === '3-link' ? 3 : 6;
    const localX = target[0] - base[0];
    const localY = target[1] - base[1];
    const distance = Math.sqrt(localX * localX + localY * localY);
    const angle = Math.atan2(localY, localX);
    
    if (armType === '2-link') {
      const L1 = config.arm_params.L1;
      const L2 = config.arm_params.L2;
      const clampedDist = Math.min(distance, L1 + L2 - 0.01);
      const cosAngle2 = (clampedDist * clampedDist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
      const angle2 = Math.acos(Math.max(-1, Math.min(1, cosAngle2)));
      const angle1 = angle - Math.atan2(L2 * Math.sin(angle2), L1 + L2 * Math.cos(angle2));
      return [angle1, angle2];
    } else {
      return Array(numJoints).fill(0).map((_, i) => 
        angle / numJoints + (i > 0 ? 0.2 : 0)
      );
    }
  };
  
  // Get "idle" configuration (pointing north)
  const getIdleConfig = () => {
    const numJoints = armType === '2-link' ? 2 : armType === '3-link' ? 3 : 6;
    return Array(numJoints).fill(0); // All joints at 0 = pointing up
  };
  
  // Update separation when bases change (no automatic arm gripping)
  useEffect(() => {
    setConfig(prev => ({
      ...prev,
      separation: Math.abs(rightBaseX - leftBaseX)
    }));
  }, [leftBaseX, rightBaseX]);
  
  // Handle mouse events for dragging
  const handleMouseDown = (e) => {
    if (isPlanning) return;
    
    const rect = canvas.canvasRef.current.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    const worldPos = screenToWorld(
      [screenX, screenY],
      canvas.dimensions.width,
      canvas.dimensions.height,
      { minX: -4, maxX: 4, minY: -2, maxY: 4 }
    );
    
    const threshold = 0.25;
    const baseThreshold = 0.2;
    
    // Check if clicking on bases (higher priority)
    if (Math.hypot(worldPos[0] - leftBase[0], worldPos[1] - leftBase[1]) < baseThreshold) {
      setDragging('base-left');
    } else if (Math.hypot(worldPos[0] - rightBase[0], worldPos[1] - rightBase[1]) < baseThreshold) {
      setDragging('base-right');
    }
    // Check if clicking on item start or goal
    else if (Math.hypot(worldPos[0] - itemStartPos[0], worldPos[1] - itemStartPos[1]) < threshold) {
      setDragging('item-start');
    } else if (Math.hypot(worldPos[0] - itemGoalPos[0], worldPos[1] - itemGoalPos[1]) < threshold) {
      setDragging('item-goal');
    }
  };
  
  const handleMouseMove = (e) => {
    if (!dragging) return;
    
    const rect = canvas.canvasRef.current.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    const worldPos = screenToWorld(
      [screenX, screenY],
      canvas.dimensions.width,
      canvas.dimensions.height,
      { minX: -4, maxX: 4, minY: -2, maxY: 4 }
    );
    
    // Update position based on what's being dragged
    if (dragging === 'item-start') {
      setItemStartPos(worldPos);
    } else if (dragging === 'item-goal') {
      setItemGoalPos(worldPos);
    } else if (dragging === 'base-left') {
      setLeftBaseX(worldPos[0]);
    } else if (dragging === 'base-right') {
      setRightBaseX(worldPos[0]);
    }
  };
  
  const handleMouseUp = () => {
    setDragging(null);
  };
  
  // Get current display data (saved or fresh)
  const displayTree = resultMode === 'saved' && savedResults?.tree ? savedResults.tree : tree;
  const displayPath = resultMode === 'saved' && savedResults?.path ? { path: savedResults.path } : path;
  const displayStrategy = resultMode === 'saved' && savedResults?.strategy ? savedResults.strategy : strategy;
  const displayHandoffPoint = resultMode === 'saved' && savedResults?.handoff_point ? savedResults.handoff_point : handoffPoint;
  const displayPhases = resultMode === 'saved' && savedResults?.phases ? savedResults.phases : phases;
  
  // Draw visualization
  useEffect(() => {
    if (!canvas.canvasRef.current) return;
    
    canvas.clear();
    
    // Draw obstacles
    drawObstacles(canvas, config.obstacles);
    
    // Draw tree if available and enabled
    if (showTree && displayTree && displayTree.nodes && displayTree.nodes.length > 0) {
      drawTree(
        canvas,
        displayTree,
        armType,
        config.arm_params,
        leftBase,
        rightBase
      );
    }
    
    // Draw path if available
    if (displayPath && displayPath.path && displayPath.path.length > 0) {
      drawPath(canvas, displayPath);
    }
    
    // Draw current arm configuration
    // If planning is complete and we have a path, show final configuration
    // Otherwise, show the start configuration (arms pointing north)
    const displayConfig = (status === 'completed' || resultMode === 'saved') && displayPath && displayPath.path && displayPath.path.length > 0 ? 
      displayPath.path[displayPath.path.length - 1].config : 
      config.start;
    
    drawArms(
      canvas,
      armType,
      displayConfig,
      config.arm_params,
      leftBase,
      rightBase
    );
    
    // Draw item markers (what the arms will manipulate)
    // Start item (green)
    const itemSize = 0.15;
    canvas.drawCircle(itemStartPos, itemSize, '#22c55e', true);
    canvas.drawText('START', [itemStartPos[0], itemStartPos[1] - 0.3], '#22c55e', 12);
    
    // Goal item (yellow - hollow circle)
    canvas.drawCircle(itemGoalPos, itemSize, '#fbbf24', false);
    const ctx = canvas.getContext();
    if (ctx) {
      const [x, y, scale] = canvas.toScreen(itemGoalPos);
      ctx.beginPath();
      ctx.arc(x, y, itemSize * scale, 0, 2 * Math.PI);
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.fillStyle = '#fbbf2433';
      ctx.fill();
    }
    canvas.drawText('GOAL', [itemGoalPos[0], itemGoalPos[1] - 0.3], '#fbbf24', 12);
    
    // Draw handoff point if available (with enhanced visualization)
    if (displayHandoffPoint) {
      // Draw connection lines
      canvas.drawLine(leftBase, displayHandoffPoint, '#9333ea33', 2);
      canvas.drawLine(rightBase, displayHandoffPoint, '#9333ea33', 2);
      
      // Draw handoff point with glow effect
      canvas.drawCircle(displayHandoffPoint, 0.25, '#9333ea44', true);
      canvas.drawCircle(displayHandoffPoint, 0.18, '#9333ea', true);
      canvas.drawCircle(displayHandoffPoint, 0.12, '#a855f7', true);
      
      // Label
      canvas.drawText('HANDOFF', [displayHandoffPoint[0], displayHandoffPoint[1] - 0.4], '#a855f7', 14, 'bold');
    }
    
    // Draw bases (larger and more visible for dragging)
    canvas.drawCircle(leftBase, 0.18, '#3b82f6', true);
    canvas.drawCircle(rightBase, 0.18, '#ef4444', true);
    canvas.drawText('LEFT', [leftBase[0], leftBase[1] - 0.3], '#3b82f6', 12);
    canvas.drawText('RIGHT', [rightBase[0], rightBase[1] - 0.3], '#ef4444', 12);
    
  }, [canvas, config, armType, displayTree, displayPath, status, leftBase, rightBase, dragging, showTree, displayHandoffPoint, resultMode]);
  
  const isPlanning = status === 'running' || status === 'starting';
  
  return (
    <div className="app flex flex-col h-screen">
      {/* Demo Navigation Bar */}
      <DemoNavBar
        demos={demos}
        activeDemoId={activeDemoId}
        onSelectDemo={handleSelectDemo}
        isPlanning={isPlanning}
      />
      
      <div className="flex flex-1 overflow-hidden">
      <Sidebar>
        <StatusIndicator status={status} error={error} progress={progress} />
        
        <ArmSelector 
          value={armType} 
          onChange={handleArmTypeChange}
          disabled={isPlanning}
        />
        
        <ConfigPanel 
          config={config} 
          onChange={setConfig}
          disabled={isPlanning}
        />
        
        <OptimizationToggles 
          config={config} 
          onChange={setConfig}
          disabled={isPlanning}
        />
        
        <StartStopButton
          state={status}
          onStart={handleStartPlanning}
          onStop={stopPlanning}
        />
        
        <ClearButton
          onClear={handleClear}
          disabled={status === 'running' || status === 'starting'}
        />
        
        <MetricsDisplay 
          progress={resultMode === 'saved' && savedResults?.metrics ? savedResults.metrics : progress} 
          path={displayPath}
          planningTime={resultMode === 'saved' && savedResults?.metrics ? savedResults.metrics.planning_time : progress?.planning_time}
          status={resultMode === 'saved' ? 'completed' : status}
        />
        
        {/* Demo Controls */}
        <DemoControls
          activeDemoId={activeDemoId}
          resultMode={resultMode}
          onResetToSaved={handleResetToSaved}
          onRerunFresh={handleRerunFresh}
          onSaveAsDemo={handleSaveAsDemo}
          isPlanning={isPlanning}
          hasResults={path || savedResults}
        />
        
        {/* Handoff Plan Display */}
        {displayStrategy && (
          <div className="bg-gradient-to-br from-purple-900/40 to-blue-900/40 border border-purple-500/30 rounded-lg p-4 space-y-3">
            <h3 className="text-base font-bold text-purple-300 flex items-center gap-2">
              <span className="text-lg">🤝</span>
              Handoff Plan
            </h3>
            
            {displayStrategy.needs_handoff ? (
              <div className="space-y-2">
                {/* Grabber Arm */}
                <div className="bg-black/30 rounded p-2 border-l-4 border-green-500">
                  <div className="text-xs text-gray-400 uppercase tracking-wide">Grabber</div>
                  <div className="text-sm font-semibold text-green-400 capitalize">
                    {displayStrategy.grab_arm} Arm
                  </div>
                  <div className="text-xs text-gray-300 mt-1">
                    Picks up item from START
                  </div>
                </div>
                
                {/* Handoff Location */}
                {displayHandoffPoint && (
                  <div className="bg-black/30 rounded p-2 border-l-4 border-purple-500">
                    <div className="text-xs text-gray-400 uppercase tracking-wide">Handoff Point</div>
                    <div className="text-sm font-semibold text-purple-400">
                      [{displayHandoffPoint[0].toFixed(2)}, {displayHandoffPoint[1].toFixed(2)}]
                    </div>
                    <div className="text-xs text-gray-300 mt-1">
                      Transfer location in workspace intersection
                    </div>
                  </div>
                )}
                
                {/* Delivery Arm */}
                <div className="bg-black/30 rounded p-2 border-l-4 border-yellow-500">
                  <div className="text-xs text-gray-400 uppercase tracking-wide">Delivery</div>
                  <div className="text-sm font-semibold text-yellow-400 capitalize">
                    {displayStrategy.delivery_arm} Arm
                  </div>
                  <div className="text-xs text-gray-300 mt-1">
                    Delivers item to GOAL
                  </div>
                </div>
                
                {/* Phase Summary */}
                {displayPhases && displayPhases.length > 0 && (
                  <div className="pt-2 border-t border-purple-500/30">
                    <div className="flex justify-between text-xs">
                      <span className="text-gray-400">Total Phases:</span>
                      <span className="text-purple-300 font-semibold">{displayPhases.length}</span>
                    </div>
                    <div className="flex justify-between text-xs mt-1">
                      <span className="text-gray-400">Total Waypoints:</span>
                      <span className="text-purple-300 font-semibold">
                        {displayPhases.reduce((sum, p) => sum + p.path_length, 0)}
                      </span>
                    </div>
                  </div>
                )}
              </div>
            ) : (
              <div className="space-y-2">
                {/* Single Arm Solution */}
                <div className="bg-black/30 rounded p-2 border-l-4 border-blue-500">
                  <div className="text-xs text-gray-400 uppercase tracking-wide">Strategy</div>
                  <div className="text-sm font-semibold text-blue-400">
                    Single Arm Solution
                  </div>
                  <div className="text-xs text-gray-300 mt-1 capitalize">
                    Using {displayStrategy.chosen_arm} arm only
                  </div>
                </div>
                
                <div className="text-xs text-gray-400 bg-black/20 rounded p-2">
                  ℹ️ No handoff needed - one arm can reach both start and goal positions
                </div>
              </div>
            )}
          </div>
        )}
        
        {/* Tree visualization toggle */}
        <div className="space-y-2">
          <h3 className="text-sm font-semibold text-gray-300">Visualization</h3>
          <label className="flex items-center justify-between p-2 rounded hover:bg-gray-700 cursor-pointer">
            <span className="text-sm text-gray-300">Show RRT* Tree</span>
            <input
              type="checkbox"
              checked={showTree}
              onChange={(e) => setShowTree(e.target.checked)}
              className="w-5 h-5 accent-blue-500"
            />
          </label>
          {displayTree && displayTree.nodes && (
            <div className="text-xs text-gray-400 px-2">
              {displayTree.nodes.length} nodes explored
            </div>
          )}
        </div>
      </Sidebar>
      
      <MainCanvas>
        <canvas
          ref={canvas.canvasRef}
          width={canvas.dimensions.width}
          height={canvas.dimensions.height}
          className="w-full h-full"
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
          style={{ cursor: dragging ? 'grabbing' : 'crosshair' }}
        />
        
        <div className="absolute bottom-4 left-4 bg-dark-panel p-3 rounded-lg border border-gray-700 text-sm space-y-2">
          <div>
            <div className="font-semibold text-white mb-2">Setup</div>
            
            <div className="space-y-1 mb-2 pb-2 border-b border-gray-600">
              <div className="text-xs text-gray-400 mb-1">Arm Bases (Draggable)</div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 rounded-full bg-blue-500"></div>
                <span className="text-gray-300 text-xs">Left base + workspace</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 rounded-full bg-red-500"></div>
                <span className="text-gray-300 text-xs">Right base + workspace</span>
              </div>
            </div>
            
            <div className="space-y-1">
              <div className="text-xs text-gray-400 mb-1">Item Positions (Draggable)</div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 rounded-full bg-green-500"></div>
                <span className="text-gray-300 text-xs">START item</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 rounded-full bg-yellow-500 border border-yellow-500"></div>
                <span className="text-gray-300 text-xs">GOAL item</span>
              </div>
              {displayHandoffPoint && (
                <div className="flex items-center gap-2 mt-2 pt-2 border-t border-gray-600">
                  <div className="w-3 h-3 rounded-full bg-purple-500"></div>
                  <span className="text-gray-300 text-xs">Handoff point</span>
                </div>
              )}
            </div>
          </div>
          
          {showTree && displayTree && displayTree.nodes && displayTree.nodes.length > 0 && (
            <div className="pt-2 border-t border-gray-600">
              <div className="font-semibold text-white text-xs mb-1">RRT* Tree</div>
              <div className="text-xs text-gray-400">Exploration paths shown</div>
            </div>
          )}
        </div>
      </MainCanvas>
      </div>
    </div>
  );
}

export default App;
