const express = require('express');
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');
const cors = require('cors');
const app = express();
const PORT = process.env.PORT || 3000;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.static('public'));

// Store simulation progress
let simulationProgress = {
  running: false,
  percent: 0,
  status: 'idle'
};

// Route to serve the main page
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Route to start a simulation
app.post('/api/simulate', (req, res) => {
  const {
    width,
    height,
    xdim,
    ydim,
    velocity,
    viscosity,
    timeStepsPerFrame,
    totalTimeSteps,
    gifDelay
  } = req.body;
  
  // Already running?
  if (simulationProgress.running) {
    return res.status(409).json({ 
      error: 'Simulation already running',
      progress: simulationProgress
    });
  }
  
  // Reset progress
  simulationProgress = {
    running: true,
    percent: 0,
    status: 'starting'
  };
  
  // Create public directory if it doesn't exist
  if (!fs.existsSync('public')) {
    fs.mkdirSync('public');
  }
  
  // Create temporary configuration file
  const configPath = path.join(__dirname, 'temp_config.json');
  fs.writeFileSync(configPath, JSON.stringify({
    width,
    height,
    xdim,
    ydim,
    velocity,
    viscosity,
    timeStepsPerFrame,
    totalTimeSteps,
    gifDelay
  }, null, 2));
  
  // Response to client
  res.json({ 
    message: 'Simulation started',
    progress: simulationProgress
  });
  
  // Determine the correct executable name based on platform
  const executable = process.platform === 'win32' ? 'cfd_simulator.exe' : './cfd_simulator';
  
  // Execute Go binary with config file
  const simulation = spawn(executable, [`-config=${configPath}`]);
  
  // Track progress from stdout
  simulation.stdout.on('data', (data) => {
    const dataStr = data.toString();
    console.log(dataStr);
    
    if (dataStr.includes('Progress:')) {
      const match = dataStr.match(/Progress: (\d+)%/);
      if (match && match[1]) {
        simulationProgress.percent = parseInt(match[1]);
        simulationProgress.status = 'running';
        console.log(`Simulation progress: ${simulationProgress.percent}%`);
      }
    }
  });
  
  // Handle errors
  simulation.stderr.on('data', (data) => {
    console.error(`Simulation error: ${data}`);
    simulationProgress.status = 'error';
  });
  
  // Handle completion
  simulation.on('close', (code) => {
    console.log(`Simulation exited with code ${code}`);
    
    if (code === 0) {
      simulationProgress.percent = 100;
      simulationProgress.status = 'complete';
    } else {
      simulationProgress.status = 'error';
    }
    
    simulationProgress.running = false;
    
    // Clean up config
    try {
      fs.unlinkSync(configPath);
    } catch (err) {
      console.error('Error removing config file:', err);
    }
  });
});

// Route to check simulation progress
app.get('/api/progress', (req, res) => {
  res.json(simulationProgress);
});

// Route to get the simulation result
app.get('/api/result', (req, res) => {
  const gifPath = path.join(__dirname, 'public', 'cfd_simulation.gif');
  
  if (fs.existsSync(gifPath)) {
    res.sendFile(gifPath);
  } else {
    res.status(404).json({ error: 'Simulation result not found' });
  }
});

// Start server
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
  console.log(`Open your browser to http://localhost:${PORT}`);
});