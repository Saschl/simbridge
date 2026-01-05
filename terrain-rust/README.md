# SimBridge Terrain Service (Rust)

A Rust implementation of the FlyByWire SimBridge terrain rendering service for Microsoft Flight Simulator.

## Features

- **CPU-based terrain rendering** - No GPU required, runs on any system
- **REST API compatible** - Same API as the original TypeScript implementation
- **SimConnect integration** - Receives aircraft status directly from MSFS
- **Efficient terrain tile caching** - Loads and caches terrain data on demand
- **Navigation display rendering** - Supports both ARC and scanline modes
- **Vertical display support** - Renders terrain profile along flight path

## Building

### Prerequisites

- Rust 1.70 or later
- Windows (for SimConnect support)

### Build Commands

```bash
# Debug build
cargo build

# Release build (optimized)
cargo build --release
```

## Running

The service expects the `terrain.map` file to be located at:
- `./terrain/terrain.map` (relative to the executable)

### Start the service

```bash
cargo run --release
```

The HTTP server will start on port 8380.

## API Endpoints

### GET /health
Health check endpoint.

### GET /api/v1/terrain/renderingTimestamp?display=L|R
Returns the timestamp of the current rendering data for the specified display.

### GET /api/v1/terrain/renderingThresholds?display=L|R
Returns the elevation thresholds for the current rendering data.

### GET /api/v1/terrain/renderingFrames?display=L|R
Returns base64-encoded PNG frames for the current terrain render.

### POST /api/v1/terrain/aircraftStatusData
Update aircraft status data. Body should be JSON with aircraft position, altitude, EFIS settings, etc.

### POST /api/v1/terrain/verticalDisplayPath
Update the vertical display path with flight plan waypoints.

## Architecture

```
terrain-rust/
├── src/
│   ├── main.rs              # Entry point and server setup
│   ├── types.rs             # Core type definitions
│   ├── web.rs               # HTTP API handlers
│   ├── simconnect_handler.rs # SimConnect communication
│   ├── fileformat/          # Terrain file parsing
│   │   ├── mod.rs
│   │   ├── terrainmap.rs    # Main terrain map file
│   │   └── tile.rs          # Individual tile handling
│   ├── mapdata/             # Map data structures
│   │   ├── mod.rs
│   │   ├── tilemanager.rs   # Tile cache management
│   │   └── worldmap.rs      # World map operations
│   └── processing/          # Terrain rendering
│       ├── mod.rs           # Main processor
│       ├── renderer.rs      # Display renderers
│       └── patterns.rs      # Scan patterns
```

## SimConnect Communication

The service communicates with MSFS via SimConnect client data areas:

### Client Data Areas (Receiving from MSFS)
- **FBW_SIMBRIDGE_EGPWC_AIRCRAFT_STATUS** - Receives aircraft position, altitude, heading, vertical speed, gear status, runway data, and EFIS settings (46 bytes, binary packed)

### Client Data Areas (Sending to MSFS)
- **FBW_SIMBRIDGE_TERRONND_THRESHOLDS_LEFT** - Terrain metadata for left display (14 bytes)
- **FBW_SIMBRIDGE_TERRONND_THRESHOLDS_RIGHT** - Terrain metadata for right display (14 bytes)
- **FBW_SIMBRIDGE_TERRONND_FRAME_DATA_LEFT** - Rendered frame data for left display (max 8KB chunks)
- **FBW_SIMBRIDGE_TERRONND_FRAME_DATA_RIGHT** - Rendered frame data for right display (max 8KB chunks)

### System Events
- **Sim** - Subscribed to detect simulator start/stop
- **Pause_EX1** - Subscribed to detect pause state changes

### Connection Behavior
- Automatically attempts to connect to MSFS on startup
- Retries connection every 10 seconds if MSFS is not running
- Reconnects automatically if connection is lost
- Pauses terrain rendering when simulator is paused

## Terrain Data Format

The `terrain.map` file format:
- **Header (14 bytes)**: Latitude/longitude ranges, angular steps, resolution
- **Tiles**: Gzip-compressed elevation data for each geographic tile

Each tile contains:
- Grid dimensions (rows, columns)
- Southwest corner coordinates
- Compressed elevation data (meters, converted to feet on load)

## Differences from TypeScript Version

1. **No GPU acceleration** - All rendering is CPU-based
2. **Simplified pattern rendering** - Basic scan patterns without GPU shaders
3. **Single-threaded rendering** - Uses async web framework but rendering is synchronous
4. **Native SimConnect** - Uses Rust simconnect crate instead of Node.js bindings

## License

Same license as the main SimBridge project.
