package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/gif"
	"image/png"
	"io/ioutil"
	"log"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"sync"
)

// Configuration - loaded from JSON
type Config struct {
	// Canvas dimensions
	Width  int `json:"width"`
	Height int `json:"height"`

	// Simulation dimensions
	XDim int `json:"xdim"`
	YDim int `json:"ydim"`

	// Physics parameters
	Velocity  float64 `json:"velocity"`
	Viscosity float64 `json:"viscosity"`

	// Simulation control
	TimeStepsPerFrame int `json:"timeStepsPerFrame"`
	TotalTimeSteps    int `json:"totalTimeSteps"`
	GifDelay          int `json:"gifDelay"`

	// Obstacle configuration
	Obstacle ObstacleConfig `json:"obstacle"`
	// Visualization options
	VisualizeVelocity bool `json:"visualizeVelocity"`
	VisualizePressure bool `json:"visualizePressure"`
	SideBySide        bool `json:"sideBySide"`
}

type ObstacleConfig struct {
	Type string   `json:"type"`
	Data [][]bool `json:"data,omitempty"`
}

var defaultConfig = Config{
	Width:             1200,
	Height:            480,
	XDim:              600,
	YDim:              240,
	Velocity:          0.10,
	Viscosity:         0.020,
	TimeStepsPerFrame: 20,
	TotalTimeSteps:    20000,
	GifDelay:          2,
	Obstacle: ObstacleConfig{
		Type: "circle",
	},
	VisualizeVelocity: true,
	VisualizePressure: true,
	SideBySide:        true,
}

// Output path for the GIF file
const outputGifPath = "public/cfd_simulation.gif"

// CFDSimulation represents the fluid dynamics simulation
type CFDSimulation struct {
	// Config parameters
	config Config

	// Density distributions in 9 directions
	n0, nN, nS, nE, nW, nNW, nNE, nSW, nSE [][]float64

	// Calculated fields
	density [][]float64
	xvel    [][]float64
	yvel    [][]float64
	speed2  [][]float64

	// Barrier array
	barrier [][]bool

	// Calculation shortcuts
	four9ths float64
	one9th   float64
	one36th  float64
	pressure [][]float64

	// Simulation state
	timestep int
	mutex    sync.RWMutex
	running  bool

	// GIF related
	frames []*image.Paletted
	delays []int
}

// NewCFDSimulation creates a new simulation instance with the provided config
func NewCFDSimulation(config Config) *CFDSimulation {
	sim := &CFDSimulation{
		config:   config,
		four9ths: 4.0 / 9,
		one9th:   1.0 / 9,
		one36th:  1.0 / 36,
		running:  true,
		frames:   make([]*image.Paletted, 0),
		delays:   make([]int, 0),
	}

	// Initialize all arrays
	sim.initArrays()
	sim.reset()

	return sim
}

func nacaAirfoil(x, y, xdim, ydim int) bool {
	// Normalize coordinates
	xf := float64(x) / float64(xdim)
	yf := float64(y) / float64(ydim)

	// Airfoil position
	xOffset := 0.001
	yOffset := 0.5
	xn := xf - xOffset
	yn := yf - yOffset

	// NACA 4-digit parameters (example: NACA 2412)
	m := 0.02 // 2% camber (2/100)
	p := 0.5  // Max camber at 40% chord (4/10)
	t := 0.3  // 12% thickness (12/100)

	// Reject points outside chord
	if xn < 0.0 || xn > 1.0 {
		return false
	}

	// Camber line and slope
	var yc, dycdx float64
	if xn < p {
		yc = (m / (p * p)) * (2*p*xn - xn*xn)
		dycdx = (2 * m / (p * p)) * (p - xn)
	} else {
		yc = (m / ((1 - p) * (1 - p))) * ((1 - 2*p) + 2*p*xn - xn*xn)
		dycdx = (2 * m / ((1 - p) * (1 - p))) * (p - xn)
	}

	theta := math.Atan(dycdx)

	// Thickness distribution (NACA 00xx thickness formula)
	yt := 5 * t * (0.2969*math.Sqrt(xn) -
		0.1260*xn -
		0.3516*xn*xn +
		0.2843*xn*xn*xn -
		0.1015*xn*xn*xn*xn)

	// Upper and lower surface positions
	yUpper := yc + yt*math.Cos(theta)
	yLower := yc - yt*math.Cos(theta)

	// Adjust for better visualization scale
	scaleFactor := 0.8
	yUpper = yc + scaleFactor*(yUpper-yc)
	yLower = yc + scaleFactor*(yLower-yc)

	// Check if point is between upper and lower surface
	return yn >= yLower && yn <= yUpper
}

// initArrays initializes all 2D arrays according to configuration
func (sim *CFDSimulation) initArrays() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	// Initialize density distribution arrays
	sim.n0 = make([][]float64, xdim)
	sim.nN = make([][]float64, xdim)
	sim.nS = make([][]float64, xdim)
	sim.nE = make([][]float64, xdim)
	sim.nW = make([][]float64, xdim)
	sim.nNW = make([][]float64, xdim)
	sim.nNE = make([][]float64, xdim)
	sim.nSW = make([][]float64, xdim)
	sim.nSE = make([][]float64, xdim)

	// Initialize calculated arrays
	sim.density = make([][]float64, xdim)
	sim.xvel = make([][]float64, xdim)
	sim.yvel = make([][]float64, xdim)
	sim.speed2 = make([][]float64, xdim)

	// Initialize barrier array
	sim.barrier = make([][]bool, xdim)

	sim.pressure = make([][]float64, xdim)
	for x := 0; x < xdim; x++ {
		sim.pressure[x] = make([]float64, ydim)
	}

	// Allocate inner arrays
	for x := 0; x < xdim; x++ {
		sim.n0[x] = make([]float64, ydim)
		sim.nN[x] = make([]float64, ydim)
		sim.nS[x] = make([]float64, ydim)
		sim.nE[x] = make([]float64, ydim)
		sim.nW[x] = make([]float64, ydim)
		sim.nNW[x] = make([]float64, ydim)
		sim.nNE[x] = make([]float64, ydim)
		sim.nSW[x] = make([]float64, ydim)
		sim.nSE[x] = make([]float64, ydim)

		sim.density[x] = make([]float64, ydim)
		sim.xvel[x] = make([]float64, ydim)
		sim.yvel[x] = make([]float64, ydim)
		sim.speed2[x] = make([]float64, ydim)

		sim.barrier[x] = make([]bool, ydim)
	}
}

// Add this improved version of the initializeBarrier function to your main.go

// Improved initializeBarrier function to better handle custom obstacles

func (sim *CFDSimulation) initializeBarrier() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	// Clear existing barrier
	for x := 0; x < xdim; x++ {
		for y := 0; y < ydim; y++ {
			sim.barrier[x][y] = false
		}
	}

	obstacleType := sim.config.Obstacle.Type
	log.Printf("Initializing barrier with obstacle type: %s", obstacleType)

	switch obstacleType {
	case "circle":
		// Create circular barrier
		for x := 0; x < xdim; x++ {
			for y := 0; y < ydim; y++ {
				relx := xdim/2 - x
				rely := ydim/2 - y
				r := math.Sqrt(float64(relx*relx + rely*rely))

				minDim := xdim
				if ydim < xdim {
					minDim = ydim
				}
				sim.barrier[x][y] = (r < float64(minDim)*0.2)
			}
		}
		log.Printf("Created circular obstacle")

	case "square":
		// Create square barrier
		size := int(math.Min(float64(xdim), float64(ydim)) * 0.2)
		centerX := xdim / 2
		centerY := ydim / 2

		for x := centerX - size; x <= centerX+size; x++ {
			for y := centerY - size; y <= centerY+size; y++ {
				if x >= 0 && x < xdim && y >= 0 && y < ydim {
					sim.barrier[x][y] = true
				}
			}
		}
		log.Printf("Created square obstacle with size %d", size*2)

	case "airfoil":
		// Use the NACA airfoil function
		for x := 0; x < xdim; x++ {
			for y := 0; y < ydim; y++ {
				sim.barrier[x][y] = nacaAirfoil(x, y, xdim, ydim)
			}
		}
		log.Printf("Created NACA airfoil obstacle")

	case "custom":
		// Improved custom obstacle handling with better error checking
		if sim.config.Obstacle.Data != nil {
			customData := sim.config.Obstacle.Data
			dataHeight := len(customData)

			// Sanity check on the data with detailed logging
			if dataHeight <= 0 {
				log.Printf("Custom obstacle data has 0 height (nil or empty array), using circle instead")
				sim.config.Obstacle.Type = "circle"
				sim.initializeBarrier()
				return
			}

			// Check the first row
			if len(customData) == 0 || customData[0] == nil {
				log.Printf("Custom obstacle has invalid first row, using circle instead")
				sim.config.Obstacle.Type = "circle"
				sim.initializeBarrier()
				return
			}

			dataWidth := len(customData[0])
			if dataWidth <= 0 {
				log.Printf("Custom obstacle data has 0 width in first row, using circle instead")
				sim.config.Obstacle.Type = "circle"
				sim.initializeBarrier()
				return
			}

			log.Printf("Processing custom obstacle: %dx%d", dataWidth, dataHeight)

			// Check if we received valid data by sampling some values
			hasTrue := false
			hasFalse := false
			rowLengthsConsistent := true

			// Validate the data structure
			for y := 0; y < dataHeight && y < 10; y++ {
				if len(customData[y]) != dataWidth {
					log.Printf("Warning: Inconsistent row width at y=%d: expected %d, got %d",
						y, dataWidth, len(customData[y]))
					rowLengthsConsistent = false
				}

				for x := 0; x < dataWidth && x < 10; x++ {
					if customData[y][x] {
						hasTrue = true
					} else {
						hasFalse = true
					}
				}
			}

			if !rowLengthsConsistent {
				log.Printf("Warning: Custom obstacle data has inconsistent row lengths, will attempt to use anyway")
			}

			log.Printf("Data validation: hasTrue=%v, hasFalse=%v", hasTrue, hasFalse)

			// Scale and center the custom obstacle
			scaleX := float64(xdim) / float64(dataWidth)
			scaleY := float64(ydim) / float64(dataHeight)
			scale := math.Min(scaleX, scaleY) * 0.8 // Scale to 80% of available space

			offsetX := (xdim - int(float64(dataWidth)*scale)) / 2
			offsetY := (ydim - int(float64(dataHeight)*scale)) / 2

			log.Printf("Scaling custom obstacle by factor %f, offset (%d,%d)", scale, offsetX, offsetY)

			// Check if the scale is reasonable
			if scale <= 0 {
				log.Println("Invalid scale factor, using circle instead")
				sim.config.Obstacle.Type = "circle"
				sim.initializeBarrier()
				return
			}

			// Map the custom data to the barrier
			barrierCount := 0
			for y := 0; y < ydim; y++ {
				for x := 0; x < xdim; x++ {
					// Convert simulation coordinates to image coordinates
					imgX := int(float64(x-offsetX) / scale)
					imgY := int(float64(y-offsetY) / scale)

					// Check if inside image bounds
					if imgX >= 0 && imgX < dataWidth && imgY >= 0 && imgY < dataHeight {
						// Check for null values or invalid data
						if imgY < len(customData) && imgX < len(customData[imgY]) {
							sim.barrier[x][y] = customData[imgY][imgX]
							if sim.barrier[x][y] {
								barrierCount++
							}
						}
					}
				}
			}

			// Check if we got any barrier cells
			if barrierCount == 0 {
				log.Println("Warning: Custom obstacle resulted in 0 barrier cells")
				// Try inverting the image - sometimes people expect black to be flow and white to be obstacle
				log.Println("Attempting to invert the image...")

				barrierCount = 0
				for y := 0; y < ydim; y++ {
					for x := 0; x < xdim; x++ {
						// Convert simulation coordinates to image coordinates
						imgX := int(float64(x-offsetX) / scale)
						imgY := int(float64(y-offsetY) / scale)

						// Check if inside image bounds
						if imgX >= 0 && imgX < dataWidth && imgY >= 0 && imgY < dataHeight {
							// Check for null values or invalid data
							if imgY < len(customData) && imgX < len(customData[imgY]) {
								// Invert the value
								sim.barrier[x][y] = !customData[imgY][imgX]
								if sim.barrier[x][y] {
									barrierCount++
								}
							}
						}
					}
				}

				log.Printf("After inversion: %d barrier cells", barrierCount)

				// If still no barrier, create a simple default obstacle
				if barrierCount == 0 {
					log.Println("No barrier cells after inversion, creating default obstacle")
					// Create a small circle in the center
					centerX := xdim / 2
					centerY := ydim / 2
					radius := math.Min(float64(xdim), float64(ydim)) * 0.1

					for y := 0; y < ydim; y++ {
						for x := 0; x < xdim; x++ {
							dx := float64(x - centerX)
							dy := float64(y - centerY)
							distance := math.Sqrt(dx*dx + dy*dy)
							sim.barrier[x][y] = distance < radius
						}
					}
					log.Printf("Created default obstacle with radius %.2f", radius)
				} else {
					log.Printf("Created custom obstacle with %d barrier cells (inverted)", barrierCount)
				}
			} else {
				log.Printf("Created custom obstacle with %d barrier cells", barrierCount)
			}

			// Save the barrier for inspection - use temp directory
			barrierImagePath := "public/barrier_visualization.png"
			if err := sim.saveBarrierImage(barrierImagePath); err != nil {
				log.Printf("Error saving barrier image: %v", err)
			} else {
				log.Printf("Barrier visualization saved to %s", barrierImagePath)
			}

		} else {
			// Fallback to circle if data is invalid
			log.Println("Invalid or missing custom obstacle data, using circle instead")
			sim.config.Obstacle.Type = "circle"
			sim.initializeBarrier() // Recursively call with updated type
		}

	default:
		// Default to circle for unknown types
		log.Printf("Unknown obstacle type '%s', using circle instead", obstacleType)
		sim.config.Obstacle.Type = "circle"
		sim.initializeBarrier() // Recursively call with updated type
	}
}

func (sim *CFDSimulation) saveBarrierImage(filename string) error {
	// Create an image to visualize the barrier
	width := sim.config.Width
	height := sim.config.Height
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	// Create a new RGBA image
	img := image.NewRGBA(image.Rect(0, 0, width, height))

	// Fill background with dark gray
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			img.Set(x, y, color.RGBA{30, 30, 30, 255})
		}
	}

	// Draw barrier cells
	scaleX := float64(width) / float64(xdim)
	scaleY := float64(height) / float64(ydim)

	for y := 0; y < ydim; y++ {
		for x := 0; x < xdim; x++ {
			if sim.barrier[x][y] {
				// Calculate pixel coordinates
				x1 := int(float64(x) * scaleX)
				y1 := int(float64(y) * scaleY)
				x2 := int(float64(x+1) * scaleX)
				y2 := int(float64(y+1) * scaleY)

				// Fill the barrier cell with white
				for px := x1; px < x2; px++ {
					for py := y1; py < y2; py++ {
						if px < width && py < height {
							img.Set(px, py, color.RGBA{220, 220, 220, 255})
						}
					}
				}
			}
		}
	}

	// Create the directory if it doesn't exist
	dir := filepath.Dir(filename)
	if err := os.MkdirAll(dir, 0755); err != nil {
		return fmt.Errorf("failed to create output directory: %w", err)
	}

	// Save the image
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	err = png.Encode(f, img)
	if err != nil {
		return err
	}
	log.Printf("Barrier visualization saved to %s", filename)

	return nil
}

func (sim *CFDSimulation) reset() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	xdim := sim.config.XDim
	ydim := sim.config.YDim
	velocity := sim.config.Velocity

	// Initialize barrier based on obstacle type
	sim.initializeBarrier()

	// Initialize fluid properties based on barrier
	for x := 0; x < xdim; x++ {
		for y := 0; y < ydim; y++ {
			if sim.barrier[x][y] {
				// Barrier cells get zero densities
				sim.n0[x][y] = 0
				sim.nE[x][y] = 0
				sim.nW[x][y] = 0
				sim.nN[x][y] = 0
				sim.nS[x][y] = 0
				sim.nNE[x][y] = 0
				sim.nNW[x][y] = 0
				sim.nSE[x][y] = 0
				sim.nSW[x][y] = 0
				sim.xvel[x][y] = 0
				sim.yvel[x][y] = 0
				sim.speed2[x][y] = 0
				sim.density[x][y] = 0
				sim.pressure[x][y] = 0
			} else {
				// Initialize fluid with initial velocity
				v := velocity
				sim.n0[x][y] = sim.four9ths * (1 - 1.5*v*v)
				sim.nE[x][y] = sim.one9th * (1 + 3*v + 3*v*v)
				sim.nW[x][y] = sim.one9th * (1 - 3*v + 3*v*v)
				sim.nN[x][y] = sim.one9th * (1 - 1.5*v*v)
				sim.nS[x][y] = sim.one9th * (1 - 1.5*v*v)
				sim.nNE[x][y] = sim.one36th * (1 + 3*v + 3*v*v)
				sim.nSE[x][y] = sim.one36th * (1 + 3*v + 3*v*v)
				sim.nNW[x][y] = sim.one36th * (1 - 3*v + 3*v*v)
				sim.nSW[x][y] = sim.one36th * (1 - 3*v + 3*v*v)
				sim.density[x][y] = 1
				sim.xvel[x][y] = v
				sim.yvel[x][y] = 0
				sim.speed2[x][y] = v * v
				sim.pressure[x][y] = 1.0 / 3.0 // Initial pressure (c_s^2 * rho)
			}
		}
	}
	sim.timestep = 0
}

// advance performs one simulation step
func (sim *CFDSimulation) advance() {
	defer func() {
		if r := recover(); r != nil {
			log.Printf("Panic in advance: %v", r)
		}
	}()

	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	sim.collide()
	sim.stream()
	sim.bounce()
	sim.timestep++
}

// parallelAdvance performs multiple simulation steps with parallel processing
func (sim *CFDSimulation) parallelAdvance() {
	defer func() {
		if r := recover(); r != nil {
			log.Printf("Panic in parallelAdvance: %v", r)
		}
	}()

	if !sim.running {
		return
	}

	// Do multiple timesteps per frame
	for i := 0; i < sim.config.TimeStepsPerFrame; i++ {
		sim.mutex.Lock()
		sim.parallelCollide()
		sim.stream()
		sim.bounce()
		sim.timestep++
		sim.mutex.Unlock()
	}
}

func (sim *CFDSimulation) collide() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim
	omega := 1 / (3*sim.config.Viscosity + 0.5) // reciprocal of tau, the relaxation time

	for x := 0; x < xdim; x++ {
		for y := 0; y < ydim; y++ {
			if !sim.barrier[x][y] {
				n := sim.n0[x][y] + sim.nN[x][y] + sim.nS[x][y] + sim.nE[x][y] + sim.nW[x][y] + sim.nNW[x][y] + sim.nNE[x][y] + sim.nSW[x][y] + sim.nSE[x][y]
				sim.density[x][y] = n
				one9thn := sim.one9th * n
				one36thn := sim.one36th * n

				// Calculate velocity components
				var vx, vy float64
				if n > 0 {
					vx = (sim.nE[x][y] + sim.nNE[x][y] + sim.nSE[x][y] - sim.nW[x][y] - sim.nNW[x][y] - sim.nSW[x][y]) / n
				}
				sim.xvel[x][y] = vx

				if n > 0 {
					vy = (sim.nN[x][y] + sim.nNE[x][y] + sim.nNW[x][y] - sim.nS[x][y] - sim.nSE[x][y] - sim.nSW[x][y]) / n
				}
				sim.yvel[x][y] = vy

				// Calculate speed squared
				vx3 := 3 * vx
				vy3 := 3 * vy
				vx2 := vx * vx
				vy2 := vy * vy
				vxvy2 := 2 * vx * vy
				v2 := vx2 + vy2
				sim.speed2[x][y] = v2
				v215 := 1.5 * v2

				// Calculate pressure (p = c_s^2 * rho in LBM)
				// c_s^2 = 1/3 in the D2Q9 model
				sim.pressure[x][y] = n / 3.0

				// Existing collision code...
				sim.n0[x][y] += omega * (sim.four9ths*n*(1-v215) - sim.n0[x][y])
				sim.nE[x][y] += omega * (one9thn*(1+vx3+4.5*vx2-v215) - sim.nE[x][y])
				sim.nW[x][y] += omega * (one9thn*(1-vx3+4.5*vx2-v215) - sim.nW[x][y])
				sim.nN[x][y] += omega * (one9thn*(1+vy3+4.5*vy2-v215) - sim.nN[x][y])
				sim.nS[x][y] += omega * (one9thn*(1-vy3+4.5*vy2-v215) - sim.nS[x][y])
				sim.nNE[x][y] += omega * (one36thn*(1+vx3+vy3+4.5*(v2+vxvy2)-v215) - sim.nNE[x][y])
				sim.nNW[x][y] += omega * (one36thn*(1-vx3+vy3+4.5*(v2-vxvy2)-v215) - sim.nNW[x][y])
				sim.nSE[x][y] += omega * (one36thn*(1+vx3-vy3+4.5*(v2-vxvy2)-v215) - sim.nSE[x][y])
				sim.nSW[x][y] += omega * (one36thn*(1-vx3-vy3+4.5*(v2+vxvy2)-v215) - sim.nSW[x][y])
			}
		}
	}
}

func (sim *CFDSimulation) parallelCollide() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim
	omega := 1 / (3*sim.config.Viscosity + 0.5) // reciprocal of tau, the relaxation time

	// Split work among goroutines for better performance
	numWorkers := runtime.NumCPU()
	chunkSize := xdim / numWorkers
	var wg sync.WaitGroup

	for w := 0; w < numWorkers; w++ {
		wg.Add(1)
		go func(worker int) {
			defer wg.Done()
			defer func() {
				if r := recover(); r != nil {
					log.Printf("Panic in worker %d: %v", worker, r)
				}
			}()

			startX := worker * chunkSize
			endX := startX + chunkSize
			if worker == numWorkers-1 {
				endX = xdim // Handle remainder
			}

			for x := startX; x < endX; x++ {
				for y := 0; y < ydim; y++ {
					if !sim.barrier[x][y] {
						n := sim.n0[x][y] + sim.nN[x][y] + sim.nS[x][y] + sim.nE[x][y] + sim.nW[x][y] + sim.nNW[x][y] + sim.nNE[x][y] + sim.nSW[x][y] + sim.nSE[x][y]
						sim.density[x][y] = n
						one9thn := sim.one9th * n
						one36thn := sim.one36th * n

						var vx, vy float64
						if n > 0 {
							vx = (sim.nE[x][y] + sim.nNE[x][y] + sim.nSE[x][y] - sim.nW[x][y] - sim.nNW[x][y] - sim.nSW[x][y]) / n
						}
						sim.xvel[x][y] = vx

						if n > 0 {
							vy = (sim.nN[x][y] + sim.nNE[x][y] + sim.nNW[x][y] - sim.nS[x][y] - sim.nSE[x][y] - sim.nSW[x][y]) / n
						}
						sim.yvel[x][y] = vy

						vx3 := 3 * vx
						vy3 := 3 * vy
						vx2 := vx * vx
						vy2 := vy * vy
						vxvy2 := 2 * vx * vy
						v2 := vx2 + vy2
						sim.speed2[x][y] = v2
						v215 := 1.5 * v2

						// Calculate pressure (p = c_s^2 * rho in LBM)
						// c_s^2 = 1/3 in the D2Q9 model
						sim.pressure[x][y] = n / 3.0

						// Existing collision code...
						sim.n0[x][y] += omega * (sim.four9ths*n*(1-v215) - sim.n0[x][y])
						sim.nE[x][y] += omega * (one9thn*(1+vx3+4.5*vx2-v215) - sim.nE[x][y])
						sim.nW[x][y] += omega * (one9thn*(1-vx3+4.5*vx2-v215) - sim.nW[x][y])
						sim.nN[x][y] += omega * (one9thn*(1+vy3+4.5*vy2-v215) - sim.nN[x][y])
						sim.nS[x][y] += omega * (one9thn*(1-vy3+4.5*vy2-v215) - sim.nS[x][y])
						sim.nNE[x][y] += omega * (one36thn*(1+vx3+vy3+4.5*(v2+vxvy2)-v215) - sim.nNE[x][y])
						sim.nNW[x][y] += omega * (one36thn*(1-vx3+vy3+4.5*(v2-vxvy2)-v215) - sim.nNW[x][y])
						sim.nSE[x][y] += omega * (one36thn*(1+vx3-vy3+4.5*(v2-vxvy2)-v215) - sim.nSE[x][y])
						sim.nSW[x][y] += omega * (one36thn*(1-vx3-vy3+4.5*(v2+vxvy2)-v215) - sim.nSW[x][y])
					}
				}
			}
		}(w)
	}

	wg.Wait()
}

// stream moves particles to neighboring cells
func (sim *CFDSimulation) stream() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim
	velocity := sim.config.Velocity

	// Stream from NW corner
	for x := 0; x < xdim-1; x++ {
		for y := ydim - 1; y > 0; y-- {
			sim.nN[x][y] = sim.nN[x][y-1]
			sim.nNW[x][y] = sim.nNW[x+1][y-1]
		}
	}

	// Stream from NE corner
	for x := xdim - 1; x > 0; x-- {
		for y := ydim - 1; y > 0; y-- {
			sim.nE[x][y] = sim.nE[x-1][y]
			sim.nNE[x][y] = sim.nNE[x-1][y-1]
		}
	}

	// Stream from SE corner
	for x := xdim - 1; x > 0; x-- {
		for y := 0; y < ydim-1; y++ {
			sim.nS[x][y] = sim.nS[x][y+1]
			sim.nSE[x][y] = sim.nSE[x-1][y+1]
		}
	}

	// Stream from SW corner
	for x := 0; x < xdim-1; x++ {
		for y := 0; y < ydim-1; y++ {
			sim.nW[x][y] = sim.nW[x+1][y]
			sim.nSW[x][y] = sim.nSW[x+1][y+1]
		}
	}

	// Handle left and right edges
	for y := 0; y < ydim-1; y++ {
		sim.nS[0][y] = sim.nS[0][y+1]
	}
	for y := ydim - 1; y > 0; y-- {
		sim.nN[xdim-1][y] = sim.nN[xdim-1][y-1]
	}

	// Left boundary - incoming flow
	v := velocity
	for y := 0; y < ydim; y++ {
		if !sim.barrier[0][y] {
			sim.nE[0][y] = sim.one9th * (1 + 3*v + 3*v*v)
			sim.nNE[0][y] = sim.one36th * (1 + 3*v + 3*v*v)
			sim.nSE[0][y] = sim.one36th * (1 + 3*v + 3*v*v)
		}
	}

	// Right boundary - outflow
	for y := 0; y < ydim; y++ {
		if !sim.barrier[xdim-1][y] {
			sim.nW[xdim-1][y] = sim.one9th * (1 - 3*v + 3*v*v)
			sim.nNW[xdim-1][y] = sim.one36th * (1 - 3*v + 3*v*v)
			sim.nSW[xdim-1][y] = sim.one36th * (1 - 3*v + 3*v*v)
		}
	}

	// Top and bottom boundaries
	for x := 0; x < xdim; x++ {
		sim.n0[x][0] = sim.four9ths * (1 - 1.5*v*v)
		sim.nE[x][0] = sim.one9th * (1 + 3*v + 3*v*v)
		sim.nW[x][0] = sim.one9th * (1 - 3*v + 3*v*v)
		sim.nN[x][0] = sim.one9th * (1 - 1.5*v*v)
		sim.nS[x][0] = sim.one9th * (1 - 1.5*v*v)
		sim.nNE[x][0] = sim.one36th * (1 + 3*v + 3*v*v)
		sim.nSE[x][0] = sim.one36th * (1 + 3*v + 3*v*v)
		sim.nNW[x][0] = sim.one36th * (1 - 3*v + 3*v*v)
		sim.nSW[x][0] = sim.one36th * (1 - 3*v + 3*v*v)

		sim.n0[x][ydim-1] = sim.four9ths * (1 - 1.5*v*v)
		sim.nE[x][ydim-1] = sim.one9th * (1 + 3*v + 3*v*v)
		sim.nW[x][ydim-1] = sim.one9th * (1 - 3*v + 3*v*v)
		sim.nN[x][ydim-1] = sim.one9th * (1 - 1.5*v*v)
		sim.nS[x][ydim-1] = sim.one9th * (1 - 1.5*v*v)
		sim.nNE[x][ydim-1] = sim.one36th * (1 + 3*v + 3*v*v)
		sim.nSE[x][ydim-1] = sim.one36th * (1 + 3*v + 3*v*v)
		sim.nNW[x][ydim-1] = sim.one36th * (1 - 3*v + 3*v*v)
		sim.nSW[x][ydim-1] = sim.one36th * (1 - 3*v + 3*v*v)
	}
}

// bounce handles particle collisions with barriers
func (sim *CFDSimulation) bounce() {
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	for x := 0; x < xdim; x++ {
		for y := 0; y < ydim; y++ {
			if sim.barrier[x][y] {
				if sim.nN[x][y] > 0 && y > 0 {
					sim.nS[x][y-1] += sim.nN[x][y]
					sim.nN[x][y] = 0
				}
				if sim.nS[x][y] > 0 && y < ydim-1 {
					sim.nN[x][y+1] += sim.nS[x][y]
					sim.nS[x][y] = 0
				}
				if sim.nE[x][y] > 0 && x > 0 {
					sim.nW[x-1][y] += sim.nE[x][y]
					sim.nE[x][y] = 0
				}
				if sim.nW[x][y] > 0 && x < xdim-1 {
					sim.nE[x+1][y] += sim.nW[x][y]
					sim.nW[x][y] = 0
				}
				if sim.nNW[x][y] > 0 && x < xdim-1 && y > 0 {
					sim.nSE[x+1][y-1] += sim.nNW[x][y]
					sim.nNW[x][y] = 0
				}
				if sim.nNE[x][y] > 0 && x > 0 && y > 0 {
					sim.nSW[x-1][y-1] += sim.nNE[x][y]
					sim.nNE[x][y] = 0
				}
				if sim.nSW[x][y] > 0 && x < xdim-1 && y < ydim-1 {
					sim.nNE[x+1][y+1] += sim.nSW[x][y]
					sim.nSW[x][y] = 0
				}
				if sim.nSE[x][y] > 0 && x > 0 && y < ydim-1 {
					sim.nNW[x-1][y+1] += sim.nSE[x][y]
					sim.nSE[x][y] = 0
				}
			}
		}
	}
}

// Updated createFrame function to display velocity and pressure stacked vertically
// with larger labels
func (sim *CFDSimulation) createFrame() *image.Paletted {
	width := sim.config.Width
	height := sim.config.Height
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	// Determine if we're showing both visualizations
	showBoth := sim.config.VisualizeVelocity && sim.config.VisualizePressure && sim.config.SideBySide

	// Create color palette - make sure all entries are initialized
	palette := make(color.Palette, 256)

	// Initialize all palette entries to avoid nil entries
	// Default to black for all entries
	for i := 0; i < 256; i++ {
		palette[i] = color.RGBA{0, 0, 0, 255}
	}

	// Velocity color gradient (blues)
	for i := 0; i < 120; i++ {
		intensity := float64(i) / 119.0
		r := uint8(0)
		g := uint8(intensity * 170)
		b := uint8(64 + intensity*191)
		palette[i] = color.RGBA{r, g, b, 255}
	}

	// Pressure color gradient (reds)
	for i := 120; i < 240; i++ {
		intensity := float64(i-120) / 119.0
		r := uint8(64 + intensity*191)
		g := uint8(intensity * 80)
		b := uint8(intensity * 80)
		palette[i] = color.RGBA{r, g, b, 255}
	}

	// Reserved colors for labels
	palette[250] = color.RGBA{255, 255, 255, 255} // White
	palette[251] = color.RGBA{200, 200, 255, 255} // Light blue
	palette[252] = color.RGBA{255, 200, 200, 255} // Light red

	// Black for barriers (reused)
	palette[255] = color.RGBA{0, 0, 0, 255}

	// Create the image with the appropriate dimensions
	var img *image.Paletted
	if showBoth {
		// For vertical stacking, double the height instead of the width
		img = image.NewPaletted(image.Rect(0, 0, width, height*2), palette)
	} else {
		img = image.NewPaletted(image.Rect(0, 0, width, height), palette)
	}

	// Calculate scaling factors
	scaleX := float64(width) / float64(xdim)
	scaleY := float64(height) / float64(ydim)

	// Find min/max pressure for normalization
	minPressure := float64(1000000)
	maxPressure := float64(-1000000)

	for x := 0; x < xdim; x++ {
		for y := 0; y < ydim; y++ {
			if !sim.barrier[x][y] {
				if sim.pressure[x][y] < minPressure {
					minPressure = sim.pressure[x][y]
				}
				if sim.pressure[x][y] > maxPressure {
					maxPressure = sim.pressure[x][y]
				}
			}
		}
	}

	pressureRange := maxPressure - minPressure
	if pressureRange < 0.0001 {
		pressureRange = 0.0001 // Avoid division by zero
	}

	// Draw velocity field (top half or full image)
	if sim.config.VisualizeVelocity {
		for x := 0; x < xdim; x++ {
			for y := 0; y < ydim; y++ {
				// Calculate pixel coordinates
				x1 := int(float64(x) * scaleX)
				y1 := int(float64(y) * scaleY)
				x2 := int(float64(x+1) * scaleX)
				y2 := int(float64(y+1) * scaleY)

				// Choose color index
				var colorIdx uint8
				if sim.barrier[x][y] {
					colorIdx = 255 // Use black for barriers
				} else {
					// Map speed to color (0-119)
					speed := math.Sqrt(sim.speed2[x][y])
					intensity := math.Min(speed*3.0, 1.0)
					colorIdx = uint8(intensity * 119)
				}

				// Fill the rectangle
				for px := x1; px < x2; px++ {
					for py := y1; py < y2; py++ {
						if px < width && py < height {
							img.SetColorIndex(px, py, colorIdx)
						}
					}
				}
			}
		}

		// Add label for velocity field - make it larger
		DrawLargeLabel(img, "VELOCITY", width/2-60, 30, 251)
	}

	// Draw pressure field (bottom half or full image if only showing pressure)
	if sim.config.VisualizePressure {
		var offsetY int
		if showBoth {
			offsetY = height // Start after velocity field for vertical stacking
		} else {
			offsetY = 0 // Start at top if only showing pressure
		}

		for x := 0; x < xdim; x++ {
			for y := 0; y < ydim; y++ {
				// Calculate pixel coordinates
				x1 := int(float64(x) * scaleX)
				y1 := offsetY + int(float64(y)*scaleY)
				x2 := int(float64(x+1) * scaleX)
				y2 := offsetY + int(float64(y+1)*scaleY)

				// Choose color index
				var colorIdx uint8
				if sim.barrier[x][y] {
					colorIdx = 255 // Use black for barriers
				} else {
					// Normalize pressure to 0-1 range and map to color (120-239)
					normalizedPressure := (sim.pressure[x][y] - minPressure) / pressureRange
					colorIdx = uint8(120 + normalizedPressure*119)
				}

				// Fill the rectangle
				for px := x1; px < x2; px++ {
					for py := y1; py < y2; py++ {
						if px < width && py < (offsetY+height) {
							img.SetColorIndex(px, py, colorIdx)
						}
					}
				}
			}
		}

		// Add label for pressure field - make it larger
		if showBoth {
			// If showing both, add to the bottom half
			DrawLargeLabel(img, "PRESSURE", width/2-60, height+30, 252)

			// Add divider line
			for x := 0; x < width; x++ {
				img.SetColorIndex(x, height, 255) // Black divider line
			}
		} else {
			// If only showing pressure, add to the top
			DrawLargeLabel(img, "PRESSURE", width/2-60, 30, 252)
		}
	}

	return img
}

// Function to draw larger text labels
func DrawLargeLabel(img *image.Paletted, text string, x, y int, colorIdx uint8) {
	// Use a larger font size by increasing the scale
	scale := 2
	spacing := 12 * scale

	for i, char := range text {
		posX := x + i*spacing

		// Draw larger characters
		switch char {
		case 'V':
			for j := 0; j < 10*scale; j++ {
				offset := j * 9 * scale / (10 * scale * 2) // slope for the 'V'
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+offset+k, y+j, colorIdx)
					img.SetColorIndex(posX+(9*scale-offset)+k, y+j, colorIdx)
				}
			}
		case 'E':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
				}
			}
			for j := 0; j < 3; j++ {
				for i := 0; i < 8*scale; i++ {
					for k := 0; k < scale; k++ {
						img.SetColorIndex(posX+i+scale+k, y+j*5*scale+k, colorIdx)
					}
				}
			}
		case 'L':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+9*scale+k, colorIdx)
				}
			}
		case 'O':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
					img.SetColorIndex(posX+9*scale+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+scale+k, y+9*scale+k, colorIdx)
				}
			}
		case 'C':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+scale+k, y+9*scale+k, colorIdx)
				}
			}
		case 'I':
			for i := 0; i < 10*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+k, y+9*scale+k, colorIdx)
				}
			}
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+5*scale+k, y+j, colorIdx)
				}
			}
		case 'T':
			for i := 0; i < 10*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+k, y+k, colorIdx)
				}
			}
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+5*scale+k, y+j, colorIdx)
				}
			}
		case 'Y':
			for j := 0; j < 5*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+j+k, y+j, colorIdx)
					img.SetColorIndex(posX+9*scale-j+k, y+j, colorIdx)
				}
			}
			for j := 5 * scale; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+5*scale+k, y+j, colorIdx)
				}
			}
		case 'P':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+scale+k, y+5*scale+k, colorIdx)
				}
			}
			for j := 1; j < 5; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+9*scale+k, y+j*scale, colorIdx)
				}
			}
		case 'R':
			for j := 0; j < 10*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+scale+k, y+5*scale+k, colorIdx)
				}
			}
			for j := 1; j < 5; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+9*scale+k, y+j*scale, colorIdx)
				}
			}
			for j := 5; j < 10; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+(j-5)*scale+scale+k, y+j*scale, colorIdx)
				}
			}
		case 'S':
			for i := 0; i < 9*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+k, y+k, colorIdx)
					img.SetColorIndex(posX+i+k, y+5*scale+k, colorIdx)
					img.SetColorIndex(posX+i+k, y+9*scale+k, colorIdx)
				}
			}
			for j := 1; j < 5; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j*scale, colorIdx)
				}
			}
			for j := 6; j < 9; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+9*scale+k, y+j*scale, colorIdx)
				}
			}
		case 'U':
			for j := 0; j < 9*scale; j++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+k, y+j, colorIdx)
					img.SetColorIndex(posX+9*scale+k, y+j, colorIdx)
				}
			}
			for i := 0; i < 8*scale; i++ {
				for k := 0; k < scale; k++ {
					img.SetColorIndex(posX+i+scale+k, y+9*scale+k, colorIdx)
				}
			}
		case ' ':
			// Do nothing for space
		default:
			// Draw a simple rectangle for unknown characters
			for i := 0; i < 8*scale; i++ {
				for j := 0; j < 8*scale; j++ {
					for k := 0; k < scale; k++ {
						if i == 0 || i == 8*scale-1 || j == 0 || j == 8*scale-1 {
							img.SetColorIndex(posX+i+k, y+j+k, colorIdx)
						}
					}
				}
			}
		}
	}
}

// saveGIF saves the accumulated frames as a GIF file
func (sim *CFDSimulation) saveGIF(filename string) error {
	// Ensure the directory exists
	dir := filepath.Dir(filename)
	if err := os.MkdirAll(dir, 0755); err != nil {
		return fmt.Errorf("failed to create output directory: %w", err)
	}

	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	return gif.EncodeAll(f, &gif.GIF{
		Image: sim.frames,
		Delay: sim.delays,
	})
}

type SimulationData struct {
	Speed2            [][]float64 `json:"speed2"`
	Pressure          [][]float64 `json:"pressure"`
	Barrier           [][]bool    `json:"barrier"`
	Width             int         `json:"width"`
	Height            int         `json:"height"`
	XDim              int         `json:"xdim"`
	YDim              int         `json:"ydim"`
	GifPath           string      `json:"gifPath"`
	VisualizationType string      `json:"visualizationType"`
}

// runAndSaveSimulation runs the simulation with the specified configuration
func runAndSaveSimulation(config Config) error {
	// Create simulation
	sim := NewCFDSimulation(config)
	fmt.Println("Initializing simulation...")

	// Progress tracking
	totalFrames := config.TotalTimeSteps / config.TimeStepsPerFrame
	lastPercent := -1

	// Add initial frame
	sim.frames = append(sim.frames, sim.createFrame())
	sim.delays = append(sim.delays, config.GifDelay)

	for frame := 1; frame <= totalFrames; frame++ {
		sim.parallelAdvance()
		sim.frames = append(sim.frames, sim.createFrame())
		sim.delays = append(sim.delays, config.GifDelay)

		percent := int(float64(frame) / float64(totalFrames) * 100)
		if percent != lastPercent {
			fmt.Printf("Progress: %d%%\n", percent)
			lastPercent = percent
		}
	}

	// Save the GIF
	fmt.Println("Saving GIF...")
	err := sim.saveGIF(outputGifPath)
	if err != nil {
		return fmt.Errorf("failed to save GIF: %w", err)
	}

	fmt.Printf("Simulation complete. GIF saved to '%s'.\n", outputGifPath)
	return nil
}

func main() {
	// Parse command line flags
	configPath := flag.String("config", "", "Path to configuration file")
	flag.Parse()

	// Use default config or load from file
	config := defaultConfig

	if *configPath != "" {
		// Load config from file
		configData, err := ioutil.ReadFile(*configPath)
		if err != nil {
			log.Fatalf("Error reading config file: %v", err)
		}

		// Parse JSON config
		err = json.Unmarshal(configData, &config)
		if err != nil {
			log.Fatalf("Error parsing config file: %v", err)
		}

		fmt.Println("Loaded configuration from file")
	} else {
		fmt.Println("Using default configuration")
	}

	// Print configuration
	configJSON, _ := json.MarshalIndent(config, "", "  ")
	fmt.Printf("Configuration:\n%s\n", string(configJSON))

	// Ensure output directory exists
	dir := filepath.Dir(outputGifPath)
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		err := os.Mkdir(dir, 0755)
		if err != nil {
			log.Fatalf("Failed to create output directory: %v", err)
		}
	}

	// Run simulation
	err := runAndSaveSimulation(config)
	if err != nil {
		log.Fatalf("Error: %v", err)
	}
}
