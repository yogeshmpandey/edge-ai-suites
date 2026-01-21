# Metrics Service

A reusable, containerized microservice for system metrics collection and real-time streaming.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Features](#features)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Configuration](#configuration)
- [Deployment](#deployment)
- [Integration Guide](#integration-guide)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

The Metrics Service provides a decoupled solution for collecting, relaying, and visualizing system metrics in real-time. It acts as a WebSocket relay between metrics collectors (like Telegraf) and dashboard clients, enabling live visualization of CPU, memory, GPU, and other system metrics.

**Key benefits:**
- Fully decoupled from application services
- Deploy once, use with any frontend
- Horizontal scalability for multiple clients
- Works with standard Telegraf collectors

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Telegraf/     │────▶│  Metrics Service │────▶│  Dashboard/UI   │
│   Collector     │     │    (port 9090)   │     │   Clients       │
└─────────────────┘     └──────────────────┘     └─────────────────┘
    WebSocket              WebSocket Relay           WebSocket
   /ws/collector          Single connection        Multiple clients
```

### Data Flow

1. **Collector → Service**: Telegraf sends metrics as JSON via WebSocket to `/ws/collector`
2. **Service Processing**: Metrics are wrapped in `{"metrics": [...]}` format if needed
3. **Service → Clients**: Metrics are broadcast to all connected clients on `/ws/clients`

## Features

| Feature | Description |
|---------|-------------|
| **WebSocket Relay** | Receives metrics from collectors and broadcasts to clients in real-time |
| **Multi-Client Support** | Unlimited concurrent client connections with automatic cleanup |
| **Single Collector Lock** | Ensures only one collector can connect at a time for data integrity |
| **Optional Polling** | Can actively poll target services for additional metrics |
| **Health Monitoring** | Built-in health and status endpoints for orchestration |
| **CORS Support** | Configurable CORS for cross-origin dashboard access |
| **Container-Ready** | Optimized Docker image with non-root user |

## Quick Start

### Using Docker Compose (Recommended)

1. **Clone and navigate to the live-metrics-service directory:**
   ```bash
   cd live-metrics-service
   ```

2. **Start the service with collector:**
   ```bash
   docker compose up -d
   ```

3. **Verify the service is running:**
   ```bash
   curl http://localhost:9090/health
   # Response: {"status": "healthy"}
   ```

4. **Connect a client:**
   Open a WebSocket connection to `ws://localhost:9090/ws/clients`

### Using Docker Only

```bash
# Build the image
docker build -t live-metrics-service:latest .

# Run the container
docker run -d \
  --name live-metrics-service \
  -p 9090:9090 \
  -e LOG_LEVEL=INFO \
  live-metrics-service:latest
```

### Local Development

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -e .

# Run the service
uvicorn app.main:app --host 0.0.0.0 --port 9090 --reload
```

## API Reference

### WebSocket Endpoints

#### `WS /ws/collector`

**Purpose:** Single connection point for metrics collectors (Telegraf).

**Protocol:**
- Only one collector can connect at a time
- Receives JSON metric arrays from Telegraf
- Connection rejected with `WS_1008_POLICY_VIOLATION` if collector already connected

**Input Format (from Telegraf):**
```json
[
  {
    "name": "cpu",
    "tags": {"cpu": "cpu-total", "host": "hostname"},
    "fields": {"usage_user": 12.5},
    "timestamp": 1767758563
  }
]
```

#### `WS /ws/clients`

**Purpose:** Multiple client connections for receiving live metrics.

**Protocol:**
- Unlimited concurrent connections
- Receives metrics broadcast from collector
- Messages from clients are ignored (connection kept alive)

**Output Format (to clients):**
```json
{
  "metrics": [
    {
      "name": "cpu",
      "tags": {"cpu": "cpu-total", "host": "hostname"},
      "fields": {"usage_user": 12.5},
      "timestamp": 1767758563
    },
    {
      "name": "mem",
      "tags": {"host": "hostname"},
      "fields": {"used_percent": 45.2},
      "timestamp": 1767758563
    }
  ]
}
```

### REST Endpoints

#### `GET /health`

Basic health check.

**Response:**
```json
{"status": "healthy"}
```

#### `GET /api/health`

Detailed health with service status.

**Response:**
```json
{
  "status": "healthy",
  "collector_connected": true,
  "clients_connected": 3,
  "poller_active": false,
  "poller_target": null
}
```

#### `GET /api/metrics/status`

Full metrics collection status.

**Response:**
```json
{
  "collector_connected": true,
  "clients_connected": 3,
  "poller": {
    "active": false,
    "target": null,
    "last_metrics": null
  }
}
```

#### `GET /`

Service information and available endpoints.

**Response:**
```json
{
  "service": "Metrics Service",
  "version": "1.0.0",
  "description": "A reusable microservice for system metrics collection and relay",
  "endpoints": {
    "websocket_collector": "/ws/collector",
    "websocket_clients": "/ws/clients",
    "health": "/health",
    "health_detailed": "/api/health",
    "metrics_status": "/api/metrics/status"
  }
}
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `METRICS_PORT` | `9090` | Port to run the service on |
| `LOG_LEVEL` | `INFO` | Logging level: `DEBUG`, `INFO`, `WARNING`, `ERROR` |
| `CORS_ORIGINS` | `*` | Comma-separated list of allowed CORS origins |
| `TARGET_SERVICE_URL` | `` | Optional: URL of service to poll for metrics |
| `METRICS_ENDPOINT` | `/api/metrics/status` | Endpoint path on target service for polling |
| `POLL_INTERVAL_SECONDS` | `5` | Polling interval in seconds |

### Example Configuration

```bash
# Production settings
export METRICS_PORT=9090
export LOG_LEVEL=WARNING
export CORS_ORIGINS=https://dashboard.example.com,https://admin.example.com

# With optional polling enabled
export TARGET_SERVICE_URL=http://my-app:8080
export METRICS_ENDPOINT=/api/metrics
export POLL_INTERVAL_SECONDS=10
```

## Deployment

### Standalone Deployment

Use the included `compose.yaml` for standalone deployment with a collector:

```bash
cd live-metrics-service
docker compose up -d
```

This starts:
- **live-metrics-service**: The metrics relay service on port 9090
- **collector**: Telegraf-based collector for system metrics

### Integration with Existing Applications

Add to your existing `compose.yaml`:

```yaml
services:
  # Your existing services...
  
  live-metrics-service:
    image: ${REGISTRY:-}live-metrics-service:${TAG:-latest}
    build: ./live-metrics-service
    container_name: live-metrics-service
    ports:
      - "${METRICS_SERVICE_PORT:-9090}:9090"
    environment:
      - METRICS_PORT=9090
      - LOG_LEVEL=INFO
      - CORS_ORIGINS=*
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:9090/health"]
      interval: 30s
      timeout: 10s
      retries: 3
    networks:
      - app_network

  collector:
    image: docker.io/intel/vippet-collector:2025.2.0
    container_name: collector
    privileged: true
    pid: host
    devices:
      - "/sys:/sys"
      - "/dev:/dev"
      - "/run:/run"
      - "/proc:/proc"
    volumes:
      - "./collector/telegraf.conf:/etc/telegraf/telegraf.conf:ro"
    environment:
      - WEBSOCKET_URL=ws://live-metrics-service:9090/ws/collector
    depends_on:
      - live-metrics-service
    networks:
      - app_network
```

### Telegraf Configuration

Configure Telegraf to send metrics to the service:

```toml
# Agent configuration
[agent]
  interval = "1s"
  flush_interval = "1s"

# CPU metrics
[[inputs.cpu]]
  percpu = false
  totalcpu = true

# Memory metrics
[[inputs.mem]]

# Temperature metrics
[[inputs.temp]]

# WebSocket output to live-metrics-service
[[outputs.websocket]]
  url = "${WEBSOCKET_URL}"  # ws://live-metrics-service:9090/ws/collector
  data_format = "json"
  read_timeout = "0s"
```

## Integration Guide

### JavaScript/TypeScript Client

```javascript
class MetricsClient {
  constructor(url = 'ws://localhost:9090/ws/clients') {
    this.url = url;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.reconnectDelay = 3000;
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      console.log('Connected to metrics service');
      this.reconnectAttempts = 0;
    };

    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      this.handleMetrics(data.metrics);
    };

    this.ws.onclose = () => {
      console.log('Disconnected from metrics service');
      this.attemptReconnect();
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
  }

  handleMetrics(metrics) {
    metrics.forEach(metric => {
      switch (metric.name) {
        case 'cpu':
          console.log(`CPU: ${metric.fields.usage_user}%`);
          break;
        case 'mem':
          console.log(`Memory: ${metric.fields.used_percent}%`);
          break;
        // Handle other metrics...
      }
    });
  }

  attemptReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      setTimeout(() => this.connect(), this.reconnectDelay);
    }
  }

  disconnect() {
    if (this.ws) {
      this.ws.close();
    }
  }
}

// Usage
const client = new MetricsClient();
client.connect();
```

### Python Client

```python
import asyncio
import json
import websockets

async def metrics_client():
    uri = "ws://localhost:9090/ws/clients"
    
    async with websockets.connect(uri) as websocket:
        print("Connected to metrics service")
        
        async for message in websocket:
            data = json.loads(message)
            for metric in data.get("metrics", []):
                name = metric.get("name")
                fields = metric.get("fields", {})
                
                if name == "cpu":
                    print(f"CPU: {fields.get('usage_user', 0):.1f}%")
                elif name == "mem":
                    print(f"Memory: {fields.get('used_percent', 0):.1f}%")

# Run the client
asyncio.run(metrics_client())
```

### React Hook Example

```jsx
import { useState, useEffect, useCallback } from 'react';

function useMetrics(url = 'ws://localhost:9090/ws/clients') {
  const [metrics, setMetrics] = useState({});
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const ws = new WebSocket(url);

    ws.onopen = () => setConnected(true);
    ws.onclose = () => setConnected(false);
    
    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      const newMetrics = {};
      
      data.metrics.forEach(metric => {
        newMetrics[metric.name] = {
          ...metric.fields,
          tags: metric.tags,
          timestamp: metric.timestamp
        };
      });
      
      setMetrics(prev => ({ ...prev, ...newMetrics }));
    };

    return () => ws.close();
  }, [url]);

  return { metrics, connected };
}

// Usage in component
function Dashboard() {
  const { metrics, connected } = useMetrics();

  return (
    <div>
      <p>Status: {connected ? 'Connected' : 'Disconnected'}</p>
      <p>CPU: {metrics.cpu?.usage_user?.toFixed(1)}%</p>
      <p>Memory: {metrics.mem?.used_percent?.toFixed(1)}%</p>
    </div>
  );
}
```

## Troubleshooting

### Common Issues

#### Collector Cannot Connect

**Symptom:** Collector shows connection errors or immediately disconnects.

**Solutions:**
1. Check if another collector is already connected (only one allowed)
2. Verify the WebSocket URL is correct: `ws://live-metrics-service:9090/ws/collector`
3. Check network connectivity between containers
4. Review logs: `docker logs live-metrics-service`

#### Clients Not Receiving Metrics

**Symptom:** Client connects but receives no data.

**Solutions:**
1. Verify collector is connected: `curl http://localhost:9090/api/metrics/status`
2. Check if Telegraf is actually sending data
3. Ensure client is connected to `/ws/clients` (not `/ws/collector`)

#### High Memory Usage

**Symptom:** Service memory grows over time.

**Solutions:**
1. Check for client connection leaks (clients not properly disconnecting)
2. Monitor client count: `curl http://localhost:9090/api/health`
3. Reduce metric batch size in Telegraf configuration

### Viewing Logs

```bash
# Docker logs
docker logs live-metrics-service

# With follow
docker logs -f live-metrics-service

# Enable debug logging
docker run -e LOG_LEVEL=DEBUG live-metrics-service:latest
```

### Health Check Commands

```bash
# Basic health
curl http://localhost:9090/health

# Detailed status
curl http://localhost:9090/api/health

# Metrics status
curl http://localhost:9090/api/metrics/status
```

## License

Copyright (C) 2025 Intel Corporation  
SPDX-License-Identifier: Apache-2.0
