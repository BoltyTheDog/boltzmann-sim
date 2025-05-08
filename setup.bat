@echo off
echo Setting up Boltzmann CFD Simulator...

REM Check for Node.js and npm
where node >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo Node.js is not installed. Please install Node.js 14 or higher.
    exit /b 1
)

REM Check for Go
where go >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo Go is not installed. Please install Go 1.18 or higher.
    exit /b 1
)

REM Create public directory if it doesn't exist
if not exist "public" (
    mkdir public
    echo Created public directory
)

REM Copy index.html to public directory
copy index.html public\
echo Copied index.html to public directory

REM Install Node.js dependencies
echo Installing Node.js dependencies...
call npm install

REM Build the Go binary
echo Building Go binary...
go build -o cfd_simulator.exe main.go

REM Create default configuration
echo {> default_config.json
echo   "width": 1200,>> default_config.json
echo   "height": 480,>> default_config.json
echo   "xdim": 600,>> default_config.json
echo   "ydim": 240,>> default_config.json
echo   "velocity": 0.1,>> default_config.json
echo   "viscosity": 0.02,>> default_config.json
echo   "timeStepsPerFrame": 20,>> default_config.json
echo   "totalTimeSteps": 20000,>> default_config.json
echo   "gifDelay": 2>> default_config.json
echo }>> default_config.json
echo Created default configuration

echo Setup complete!
echo To run the simulator locally, use: npm start
echo Then open your browser to http://localhost:3000

pause