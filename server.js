const express = require('express');
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');
const cors = require('cors');
const sharp = require('sharp');
const { promisify } = require('util');
const app = express();
const PORT = process.env.PORT || 3000;

// Promisify fs.writeFile
const writeFileAsync = promisify(fs.writeFile);

// Middleware
app.use(cors());
app.use(express.json({ limit: '50mb' })); // Increased limit for image uploads
app.use(express.static('public'));

// Store simulation progress
let simulationProgress = {
  running: false,
  percent: 0,
  status: 'idle'
};

// Modified server.js to handle large obstacle images better
// Add this code to your server.js file

// 1. Add image resizing to the processObstacleImage function
async function processObstacleImage(imageData) {
  try {
    console.log("Starting image processing...");
    
    // Validate input
    if (!imageData || typeof imageData !== 'string') {
      console.error('Invalid image data format:', typeof imageData);
      throw new Error('Invalid image data');
    }
    
    // Check if the data is a proper data URL
    if (!imageData.startsWith('data:image/png;base64,')) {
      console.error('Image data is not in the expected format (data:image/png;base64,)');
      throw new Error('Invalid image data format');
    }
    
    // Remove the data URL prefix to get the base64 data
    const base64Data = imageData.replace(/^data:image\/png;base64,/, '');
    console.log(`Base64 data length: ${base64Data.length} bytes`);
    
    if (base64Data.length === 0) {
      console.error('Empty base64 data after prefix removal');
      throw new Error('Empty image data');
    }
    
    // Save the uploaded image temporarily
    const tempImagePath = path.join(__dirname, 'temp_obstacle.png');
    await writeFileAsync(tempImagePath, Buffer.from(base64Data, 'base64'));
    console.log(`Saved temporary image to ${tempImagePath}`);
    
    // Verify the file exists and has content
    const fileStats = fs.statSync(tempImagePath);
    console.log(`Temp file size: ${fileStats.size} bytes`);
    
    if (fileStats.size === 0) {
      console.error('Temporary file is empty');
      throw new Error('Failed to save image');
    }
    
    // Get image metadata first
    const metadata = await sharp(tempImagePath).metadata();
    console.log(`Original image dimensions: ${metadata.width}x${metadata.height}, channels: ${metadata.channels}`);
    
    // Resize large images to a maximum dimension of 200x200
    // This prevents performance issues with very large obstacle arrays
    const MAX_DIMENSION = 200;
    let resizedImage = sharp(tempImagePath);
    
    if (metadata.width > MAX_DIMENSION || metadata.height > MAX_DIMENSION) {
      console.log(`Image is too large, resizing to max dimension ${MAX_DIMENSION}px`);
      resizedImage = resizedImage.resize({
        width: metadata.width > metadata.height ? MAX_DIMENSION : Math.round(metadata.width * (MAX_DIMENSION / metadata.height)),
        height: metadata.height > metadata.width ? MAX_DIMENSION : Math.round(metadata.height * (MAX_DIMENSION / metadata.width)),
        fit: 'inside'
      });
    }
    
    // Ensure we have an alpha channel for transparency detection
    resizedImage = resizedImage.ensureAlpha();
    
    // Process the image to extract binary mask with enhanced error handling
    console.log("Processing with sharp...");
    
    const { data, info } = await resizedImage
      .raw()          // Get raw pixel data
      .toBuffer({ resolveWithObject: true });
    
    console.log(`Processed image dimensions: ${info.width}x${info.height}, channels: ${info.channels}`);
    console.log(`Raw data length: ${data.length} bytes`);
    
    // Create a binary obstacle array based on alpha channel
    // Pixel data is in RGBA format (4 bytes per pixel)
    const obstacleData = [];
    for (let y = 0; y < info.height; y++) {
      const row = [];
      for (let x = 0; x < info.width; x++) {
        const pixelIndex = (y * info.width + x) * 4;
        // Using alpha channel (3) to determine if pixel is an obstacle
        // If alpha is less than 128 (transparent), then it's not an obstacle
        const isObstacle = data[pixelIndex + 3] >= 128;
        row.push(isObstacle);
      }
      obstacleData.push(row);
    }
    
    console.log(`Created obstacle data array: ${obstacleData.length} rows`);
    if (obstacleData.length > 0) {
      console.log(`First row length: ${obstacleData[0].length} columns`);
    }
    
    // Clean up the temporary file
    fs.unlinkSync(tempImagePath);
    console.log("Temporary file removed");
    
    return {
      width: info.width,
      height: info.height,
      obstacleData
    };
  } catch (error) {
    console.error('Error processing obstacle image:', error);
    // If we fail to process the image, create a simple default obstacle
    // to avoid the 0 height error in the Go code
    console.log('Creating fallback obstacle data (small circle)');
    
    // Create a 32x32 array with a circle in the middle
    const fallbackSize = 32;
    const obstacleData = [];
    const center = fallbackSize / 2;
    const radius = fallbackSize / 4;
    
    for (let y = 0; y < fallbackSize; y++) {
      const row = [];
      for (let x = 0; x < fallbackSize; x++) {
        // Calculate distance from center
        const dx = x - center;
        const dy = y - center;
        const distance = Math.sqrt(dx*dx + dy*dy);
        // Inside circle = true (obstacle)
        row.push(distance < radius);
      }
      obstacleData.push(row);
    }
    
    console.log(`Created fallback obstacle: ${obstacleData.length}x${obstacleData[0].length}`);
    
    return {
      width: fallbackSize,
      height: fallbackSize,
      obstacleData
    };
  }
}


// Route to serve the main page
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Full updated app.post('/api/simulate') route with all improvements

app.post('/api/simulate', async (req, res) => {
  const {
    width,
    height,
    xdim,
    ydim,
    velocity,
    viscosity,
    timeStepsPerFrame,
    totalTimeSteps,
    gifDelay,
    obstacle
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
  
  try {
    // Initialize obstacle config with a default type
    let obstacleConfig = {
      type: 'circle' // Default is circle
    };
    
    // Update type from request if provided
    if (obstacle && obstacle.type) {
      obstacleConfig.type = obstacle.type;
      console.log(`Obstacle type set to: ${obstacleConfig.type}`);
    }
    
    // Process custom image if provided
    if (obstacleConfig.type === 'custom' && obstacle && obstacle.imageData) {
      console.log("Processing custom obstacle image...");
      const processedObstacle = await processObstacleImage(obstacle.imageData);
      
      // Validate the processed data before using it
      if (processedObstacle && 
          processedObstacle.obstacleData && 
          processedObstacle.obstacleData.length > 0 &&
          processedObstacle.obstacleData[0].length > 0) {
        
        obstacleConfig.data = processedObstacle.obstacleData;
        console.log(`Processed custom obstacle: ${processedObstacle.width}x${processedObstacle.height}`);
        
        // If obstacle is large, log a warning
        const totalCells = processedObstacle.width * processedObstacle.height;
        if (totalCells > 40000) { // 200x200
          console.log(`Warning: Large obstacle with ${totalCells} cells might affect performance`);
        }
        
        // Log obstacle data dimensions for debugging
        console.log(`Obstacle data dimensions: ${obstacleConfig.data.length}x${obstacleConfig.data[0].length}`);
      } else {
        console.error("Failed to generate valid obstacle data, falling back to circle");
        obstacleConfig.type = 'circle'; // Fall back to circle
        delete obstacleConfig.data; // Remove any invalid data
      }
    } else if (obstacleConfig.type === 'custom') {
      console.log("Custom obstacle type selected but no image data provided, falling back to circle");
      obstacleConfig.type = 'circle';
    }
    
    // Create temporary configuration file
    const configPath = path.join(__dirname, 'temp_config.json');
    const configData = {
      width,
      height,
      xdim,
      ydim,
      velocity,
      viscosity,
      timeStepsPerFrame,
      totalTimeSteps,
      gifDelay,
      obstacle: obstacleConfig
    };
    
    // Save config to file with pretty printing for readability
    fs.writeFileSync(configPath, JSON.stringify(configData, null, 2));
    console.log("Created config file at:", configPath);
    
    // Response to client
    res.json({ 
      message: 'Simulation started',
      progress: simulationProgress
    });
    
    // Verify the Go binary exists before running
    const executable = process.platform === 'win32' ? 'cfd_simulator.exe' : './cfd_simulator';
    const executablePath = path.join(__dirname, executable);
    
    if (!fs.existsSync(executablePath)) {
      console.error(`Error: Simulator executable not found at ${executablePath}`);
      simulationProgress.status = 'error';
      simulationProgress.running = false;
      return;
    }
    
    console.log(`Starting simulation: ${executablePath} -config=${configPath}`);
    
    // Execute Go binary with config file
    const simulation = spawn(executablePath, [`-config=${configPath}`]);
    
    // Add timeout to kill the process if it hangs (5 minutes)
    const timeoutId = setTimeout(() => {
      console.error('Simulation timeout after 5 minutes');
      simulation.kill();
      simulationProgress.status = 'error';
      simulationProgress.running = false;
    }, 5 * 60 * 1000);
    
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
      // Clear the timeout
      clearTimeout(timeoutId);
      
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
    
    // Add error handler for the spawn process itself
    simulation.on('error', (err) => {
      clearTimeout(timeoutId);
      console.error(`Failed to start simulation process: ${err.message}`);
      simulationProgress.status = 'error';
      simulationProgress.running = false;
    });
    
  } catch (error) {
    console.error('Simulation error:', error);
    simulationProgress.status = 'error';
    simulationProgress.running = false;
    
    // If client is still waiting for response, send error
    if (!res.headersSent) {
      res.status(500).json({ error: error.message });
    }
  }
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