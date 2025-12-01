# Dual-Arm Motion Planning - Web Application

Interactive web-based visualization for dual-arm motion planning with RRT* algorithm.

## Features

- 🎯 **Interactive Planning**: Real-time visualization of RRT* tree growth
- 🦾 **Multiple Arm Types**: Support for 2-link, 3-link, and 6-link arms
- 🎨 **Visual Feedback**: See planning progress, tree exploration, and solution paths
- ⚙️ **Configurable Parameters**: Adjust planning parameters in real-time
- 🚀 **Optimization Controls**: Toggle KD-tree search, adaptive step size, and workspace weighting
- 📊 **Performance Metrics**: View statistics on planning time, tree size, and path quality

## Architecture

```
Backend (Flask)
- Python-based REST API
- Async job processing
- Planning algorithms (RRT*, collision detection)

Frontend (React)
- Interactive visualization canvas
- Real-time status updates
- Configuration controls
```

## Installation

### Prerequisites

- **Python 3.9+** with virtual environment
- **Node.js 18+** with npm (tested on Node.js 18.20.6)
- **Backend dependencies** (Flask, NumPy, SciPy)
- **Frontend dependencies** (React 18, Vite 5, Tailwind CSS 3)

**Note**: We use Vite 5.x and React 18 for compatibility with Node.js 18. If you have Node.js 20+, these versions will still work perfectly.

### Backend Setup

1. Install Python dependencies:
```bash
./venv/bin/pip install -r backend/requirements-web.txt
```

Note: The main project dependencies should already be installed in the virtual environment.

### Frontend Setup

1. Install Node dependencies:
```bash
cd frontend
npm install
cd ..
```

## Running the Application

### Option 1: Using the Startup Script (Recommended)

```bash
chmod +x run_webapp.sh
./run_webapp.sh
```

This will start both the backend and frontend servers.

### Option 2: Manual Startup

**Terminal 1 - Start Backend:**
```bash
./venv/bin/python backend/run_server.py
```

**Terminal 2 - Start Frontend:**
```bash
cd frontend
npm run dev
```

### Accessing the Application

- **Frontend**: http://localhost:5173
- **Backend API**: http://localhost:5001/api

**Note**: We use port 5001 for the backend instead of 5000 because macOS AirPlay Receiver uses port 5000 by default.

## Using the Application

### 1. Select Arm Type

Choose from:
- **2-Link Arm** (4D configuration space)
- **3-Link Arm** (6D configuration space)
- **6-Link Arm** (12D configuration space)

### 2. Configure Planning Parameters

Adjust:
- **Max Iterations**: Maximum RRT* iterations (1000-15000)
- **Step Size**: RRT* step size (0.05-0.3)
- **Goal Threshold**: Distance threshold for goal (0.05-0.5)
- **Arm Separation**: Distance between arm bases (1.5-4.0m)

### 3. Enable Optimizations

Toggle:
- **KD-Tree Search**: Fast nearest neighbor search
- **Adaptive Step Size**: Dynamic step sizing based on obstacles
- **Workspace Weight**: Balance between joint-space and task-space distance (0-1)

### 4. Start Planning

Click "▶ Start Planning" to begin. The visualization will show:
- **RRT* Tree**: Blue (left arm) and red (right arm) exploration
- **Solution Path**: Green path when goal is reached
- **Current Configuration**: Animated arm positions

### 5. View Results

Monitor:
- **Planning Status**: Real-time status indicator
- **Progress Bar**: Iteration progress
- **Statistics**: Tree size, iterations, path length, planning time

## API Endpoints

### POST /api/plan
Start a new planning job.

**Request:**
```json
{
  "arm_type": "2-link",
  "start": [0.5, 0.5, 0.5, 0.5],
  "goal": [-0.5, -0.5, -0.5, -0.5],
  "obstacles": [
    {"type": "circle", "center": [0, 1.0], "radius": 0.3}
  ],
  "separation": 2.0,
  "max_iterations": 5000,
  "step_size": 0.15,
  "goal_threshold": 0.2,
  "use_kdtree": true,
  "workspace_weight": 0.3,
  "use_adaptive_step": true
}
```

**Response:**
```json
{
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "queued"
}
```

### GET /api/status/:job_id
Get planning job status and progress.

**Response:**
```json
{
  "job_id": "...",
  "status": "running|completed|failed",
  "progress": {
    "iterations": 1500,
    "max_iterations": 5000,
    "tree_size": 1200,
    "goal_found": false
  },
  "planning_time": 2.5
}
```

### GET /api/tree/:job_id
Get RRT* tree data for visualization.

**Response:**
```json
{
  "nodes": [
    {"id": 0, "config": [...], "parent": null, "cost": 0},
    {"id": 1, "config": [...], "parent": 0, "cost": 0.15}
  ]
}
```

### GET /api/path/:job_id
Get solution path.

**Response:**
```json
{
  "path": [
    {
      "config": [...],
      "left_ee": [x, y],
      "right_ee": [x, y]
    }
  ],
  "cost": 15.3,
  "length": 25
}
```

### POST /api/validate-config
Validate an arm configuration.

### GET /api/arm-types
Get available arm types and parameters.

### GET /api/health
Health check endpoint.

## Troubleshooting

### Backend Issues

**Port already in use:**
```bash
# Find process using port 5001
lsof -ti:5001
# Kill process
kill -9 <PID>
```

**Note**: If you get "Address already in use" for port 5001, either kill the process using it or change the port in `backend/run_server.py` and `frontend/src/utils/api.js`.

**Import errors:**
- Make sure you're using the virtual environment: `./venv/bin/python`
- Reinstall dependencies: `./venv/bin/pip install -r backend/requirements-web.txt`

### Frontend Issues

**Port already in use:**
```bash
# Find process using port 5173
lsof -ti:5173
# Kill process
kill -9 <PID>
```

**Module not found:**
```bash
cd frontend
rm -rf node_modules package-lock.json
npm install
```

**CORS errors:**
- Ensure Flask server is running
- Check that `flask-cors` is installed
- Verify API_BASE in `frontend/src/utils/api.js` matches backend URL

### Visualization Issues

**Arms not appearing:**
- Check browser console for errors
- Verify configuration dimensions match arm type
- Ensure canvas is properly sized

**Tree not showing:**
- Wait for planning to complete
- Check that job status is "completed"
- Verify tree data is received in network tab

## Development

### Hot Reload

Both servers support hot reload:
- **Backend**: Flask auto-reloads on file changes (debug mode)
- **Frontend**: Vite HMR (Hot Module Replacement)

### Adding New Features

**Backend:**
1. Add route in `backend/api/planner_api.py`
2. Update job processing in `planning_worker`
3. Test with Postman or curl

**Frontend:**
1. Create component in `frontend/src/components/`
2. Add to `App.jsx`
3. Style with Tailwind CSS classes

### Code Structure

**Backend:**
```
backend/
├── api/
│   └── planner_api.py     # Flask routes
├── requirements-web.txt   # Dependencies
└── run_server.py          # Entry point
```

**Frontend:**
```
frontend/src/
├── components/
│   ├── Canvas/            # Visualization
│   ├── Controls/          # Input controls
│   ├── Status/            # Status display
│   └── Layout/            # Layout components
├── hooks/                 # Custom React hooks
├── utils/                 # Utilities
├── App.jsx                # Main app
└── main.jsx               # Entry point
```

## Performance Tips

1. **Start with 2-link arms** for faster planning
2. **Reduce max iterations** for quick tests
3. **Enable KD-tree** for better performance
4. **Use adaptive step size** for obstacle-rich environments
5. **Adjust workspace weight** based on task requirements

## Future Enhancements

- [ ] WebSocket support for real-time updates
- [ ] Save/load configurations
- [ ] Compare multiple planning runs
- [ ] Export animations as video
- [ ] 3D visualization mode
- [ ] Multi-arm support (>2 arms)
- [ ] Interactive obstacle placement

## License

MIT License - See main project README

## Support

For issues or questions, please open an issue on GitHub.

