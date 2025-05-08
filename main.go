package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/gif"
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
}

// Default configuration
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

// reset initializes the simulation with initial conditions
func (sim *CFDSimulation) reset() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	xdim := sim.config.XDim
	ydim := sim.config.YDim
	velocity := sim.config.Velocity

	// Initialize
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

			v := velocity
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
			} else {
				// Initialize fluid with initial velocity
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

// collide performs collision step
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

// parallelCollide performs collision step using parallel processing
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

// createFrame generates a new image frame from current simulation state
func (sim *CFDSimulation) createFrame() *image.Paletted {
	// Create a paletted image for GIF
	palette := make(color.Palette, 256)
	width := sim.config.Width
	height := sim.config.Height
	xdim := sim.config.XDim
	ydim := sim.config.YDim

	// Define blue color gradient for fluid speed visualization
	for i := 0; i < 256; i++ {
		intensity := float64(i) / 255.0
		// Use blue-based HSV color scheme (similar to the canvas rendering)
		r := uint8(0)
		g := uint8(intensity * 170)    // Some green component for contrast
		b := uint8(64 + intensity*191) // Mainly blue
		palette[i] = color.RGBA{r, g, b, 255}
	}

	// Black for the barrier
	palette[255] = color.RGBA{0, 0, 0, 255}

	img := image.NewPaletted(image.Rect(0, 0, width, height), palette)

	// Draw simulation to image
	scaleX := float64(width) / float64(xdim)
	scaleY := float64(height) / float64(ydim)

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
				// Map speed to color (0-254)
				speed := math.Sqrt(sim.speed2[x][y])
				intensity := math.Min(speed*3.0, 1.0) // Same scaling as in JS
				colorIdx = uint8(intensity * 254)
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

	return img
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

// SimulationData represents the data sent to the frontend
type SimulationData struct {
	Speed2  [][]float64 `json:"speed2"`
	Barrier [][]bool    `json:"barrier"`
	Width   int         `json:"width"`
	Height  int         `json:"height"`
	XDim    int         `json:"xdim"`
	YDim    int         `json:"ydim"`
	GifPath string      `json:"gifPath"`
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
