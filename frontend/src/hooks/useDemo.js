/**
 * Custom hook for managing demo configurations and saved results
 */

import { useState, useEffect, useCallback } from 'react';
import { api } from '../utils/api';

export function useDemo() {
  const [demos, setDemos] = useState([]);
  const [activeDemoId, setActiveDemoId] = useState(null);
  const [activeDemoData, setActiveDemoData] = useState(null);
  const [resultMode, setResultMode] = useState('saved'); // 'saved' | 'fresh'
  const [savedResults, setSavedResults] = useState(null); // Store saved results separately
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Load available demos on mount
  useEffect(() => {
    loadDemos();
  }, []);

  const loadDemos = useCallback(async () => {
    try {
      setLoading(true);
      setError(null);
      const response = await api.listDemos();
      setDemos(response.demos || []);
    } catch (err) {
      console.error('Failed to load demos:', err);
      setError('Failed to load demos');
    } finally {
      setLoading(false);
    }
  }, []);

  const loadDemo = useCallback(async (demoId) => {
    try {
      setLoading(true);
      setError(null);
      
      const demoData = await api.getDemo(demoId);
      
      setActiveDemoId(demoId);
      setActiveDemoData(demoData);
      setSavedResults(demoData.saved_results || null);
      setResultMode('saved'); // Start with saved results
      
      return demoData;
    } catch (err) {
      console.error('Failed to load demo:', err);
      setError(`Failed to load demo: ${err.message}`);
      return null;
    } finally {
      setLoading(false);
    }
  }, []);

  const resetToSaved = useCallback(() => {
    if (activeDemoData && savedResults) {
      setResultMode('saved');
    }
  }, [activeDemoData, savedResults]);

  const switchToFresh = useCallback(() => {
    setResultMode('fresh');
  }, []);

  const saveAsDemo = useCallback(async (currentState) => {
    try {
      setLoading(true);
      setError(null);

      // Prompt user for demo name
      const demoName = prompt('Enter a name for this demo:');
      if (!demoName) {
        setLoading(false);
        return null;
      }

      const demoDescription = prompt('Enter a description (optional):') || '';

      const demoData = {
        metadata: {
          name: demoName,
          description: demoDescription,
          difficulty: 'custom',
          created: new Date().toISOString().replace('+00:00', 'Z'),
          version: '1.0'
        },
        config: {
          arm_type: currentState.armType,
          arm_params: currentState.armParams,
          item_start: currentState.itemStart,
          item_goal: currentState.itemGoal,
          left_base_x: currentState.leftBaseX,
          right_base_x: currentState.rightBaseX,
          obstacles: currentState.obstacles,
          rrt_params: currentState.rrtParams
        },
        saved_results: currentState.results ? {
          strategy: currentState.results.strategy,
          handoff_point: currentState.results.handoffPoint,
          phases: currentState.results.phases,
          path: currentState.results.path?.path || [],
          tree: currentState.results.tree || { nodes: [] },
          metrics: {
            planning_time: currentState.results.progress?.planning_time || 0,
            path_length: currentState.results.path?.path?.length || 0,
            tree_size: currentState.results.tree?.nodes?.length || 0,
            iterations: currentState.results.progress?.iterations || 0
          }
        } : null
      };

      const response = await api.saveDemo(demoData);
      
      // Reload demos list
      await loadDemos();
      
      alert(`Demo saved successfully as "${demoName}"!`);
      
      return response.demo_id;
    } catch (err) {
      console.error('Failed to save demo:', err);
      setError(`Failed to save demo: ${err.message}`);
      alert(`Failed to save demo: ${err.message}`);
      return null;
    } finally {
      setLoading(false);
    }
  }, [loadDemos]);

  const clearDemo = useCallback(() => {
    setActiveDemoId(null);
    setActiveDemoData(null);
    setSavedResults(null);
    setResultMode('saved');
  }, []);

  return {
    demos,
    activeDemoId,
    activeDemoData,
    resultMode,
    savedResults,
    loading,
    error,
    loadDemos,
    loadDemo,
    resetToSaved,
    switchToFresh,
    saveAsDemo,
    clearDemo
  };
}


