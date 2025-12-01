/**
 * API Client for dual-arm motion planning backend
 */

import axios from 'axios';

const API_BASE = 'http://localhost:5001/api';

export const api = {
  /**
   * Start a new planning job
   */
  startPlanning: async (config) => {
    const response = await axios.post(`${API_BASE}/plan`, config);
    return response.data;
  },
  
  /**
   * Get planning job status
   */
  getStatus: async (jobId) => {
    const response = await axios.get(`${API_BASE}/status/${jobId}`);
    return response.data;
  },
  
  /**
   * Get RRT* tree data
   */
  getTree: async (jobId) => {
    const response = await axios.get(`${API_BASE}/tree/${jobId}`);
    return response.data;
  },
  
  /**
   * Get solution path
   */
  getPath: async (jobId) => {
    const response = await axios.get(`${API_BASE}/path/${jobId}`);
    return response.data;
  },
  
  /**
   * Validate an arm configuration
   */
  validateConfig: async (config) => {
    const response = await axios.post(`${API_BASE}/validate-config`, config);
    return response.data;
  },
  
  /**
   * Get available arm types
   */
  getArmTypes: async () => {
    const response = await axios.get(`${API_BASE}/arm-types`);
    return response.data;
  },
  
  /**
   * Health check
   */
  health: async () => {
    const response = await axios.get(`${API_BASE}/health`);
    return response.data;
  },
  
  /**
   * Start planning with handoff logic
   */
  planWithHandoff: async (config) => {
    const response = await axios.post(`${API_BASE}/plan-handoff`, config);
    return response.data;
  }
};

