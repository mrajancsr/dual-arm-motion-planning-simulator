/**
 * Custom hook for interacting with the motion planner API
 */

import { useState, useCallback, useRef, useEffect } from 'react';
import { api } from '../utils/api';

export function usePlanner() {
  const [jobId, setJobId] = useState(null);
  const [status, setStatus] = useState('idle'); // idle | running | completed | failed
  const [progress, setProgress] = useState(null);
  const [tree, setTree] = useState(null);
  const [path, setPath] = useState(null);
  const [error, setError] = useState(null);
  const [strategy, setStrategy] = useState(null);
  const [handoffPoint, setHandoffPoint] = useState(null);
  const [phases, setPhases] = useState(null);
  
  const pollIntervalRef = useRef(null);
  
  const startPlanning = useCallback(async (config) => {
    try {
      setStatus('starting');
      setError(null);
      setProgress(null);
      setTree(null);
      setPath(null);
      
      // Start planning job - use handoff planning if item positions provided
      const response = config.item_start && config.item_goal
        ? await api.planWithHandoff(config)
        : await api.startPlanning(config);
      
      setJobId(response.job_id);
      setStatus('running');
      
      console.log('Planning started:', response);
      
      // Start polling for progress
      pollIntervalRef.current = setInterval(async () => {
        try {
          const statusData = await api.getStatus(response.job_id);
          
          setProgress(statusData.progress);
          
          // Update handoff-specific data if available (immediately when set)
          if (statusData.strategy) {
            setStrategy(statusData.strategy);
            console.log('Strategy updated:', statusData.strategy);
          }
          if (statusData.handoff_point) {
            setHandoffPoint(statusData.handoff_point);
            console.log('Handoff point updated:', statusData.handoff_point);
          }
          if (statusData.phases && statusData.phases.length > 0) {
            setPhases(statusData.phases);
            console.log('Phases updated:', statusData.phases);
          }
          
          // Fetch tree data while running (every poll)
          if (statusData.status === 'running') {
            try {
              const treeData = await api.getTree(response.job_id);
              if (treeData && treeData.nodes && treeData.nodes.length > 0) {
                setTree(treeData);
              }
            } catch (err) {
              // Tree might not be ready yet, that's OK
            }
          }
          
          if (statusData.status === 'completed') {
            // Get final tree and path
            const [treeData, pathData] = await Promise.all([
              api.getTree(response.job_id),
              api.getPath(response.job_id)
            ]);
            
            setTree(treeData);
            setPath(pathData);
            setStatus('completed');
            
            // Stop polling
            if (pollIntervalRef.current) {
              clearInterval(pollIntervalRef.current);
              pollIntervalRef.current = null;
            }
          } else if (statusData.status === 'failed') {
            setStatus('failed');
            setError(statusData.error || 'Planning failed');
            
            // Stop polling
            if (pollIntervalRef.current) {
              clearInterval(pollIntervalRef.current);
              pollIntervalRef.current = null;
            }
          }
        } catch (err) {
          console.error('Error polling status:', err);
        }
      }, 200); // Poll every 200ms for faster updates
      
    } catch (err) {
      setStatus('failed');
      setError(err.message || 'Failed to start planning');
    }
  }, []);
  
  const stopPlanning = useCallback(() => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current);
      pollIntervalRef.current = null;
    }
    setStatus('idle');
  }, []);
  
  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (pollIntervalRef.current) {
        clearInterval(pollIntervalRef.current);
      }
    };
  }, []);
  
  return {
    jobId,
    status,
    progress,
    tree,
    path,
    error,
    strategy,
    handoffPoint,
    phases,
    startPlanning,
    stopPlanning
  };
}

