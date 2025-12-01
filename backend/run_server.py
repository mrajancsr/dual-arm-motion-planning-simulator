#!/usr/bin/env python3
"""
Flask Server Entry Point

Starts the Flask REST API server for dual-arm motion planning visualization.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from backend.api.planner_api import app

if __name__ == '__main__':
    print("="*70)
    print("Dual-Arm Motion Planning - Flask API Server")
    print("="*70)
    print()
    print("Server starting on http://localhost:5001")
    print()
    print("API Endpoints:")
    print("  POST   /api/plan                 - Start planning")
    print("  GET    /api/status/<job_id>      - Get status")
    print("  GET    /api/tree/<job_id>        - Get tree data")
    print("  GET    /api/path/<job_id>        - Get solution path")
    print("  POST   /api/validate-config      - Validate configuration")
    print("  GET    /api/arm-types            - Get available arm types")
    print("  GET    /api/health               - Health check")
    print()
    print("="*70)
    print()
    
    app.run(debug=True, host='0.0.0.0', port=5001, threaded=True)

