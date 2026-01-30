# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

# Build stage
FROM python:3.13-slim AS builder

# Install uv for fast package installation
RUN pip install --no-cache-dir uv==0.4.23

# Set working directory
WORKDIR /app

# Copy dependency files
COPY pyproject.toml ./

# Install dependencies to a virtual environment
RUN uv venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN uv pip install --no-cache .

# Production stage
FROM python:3.13-slim AS production

RUN apt-get update && apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN groupadd --gid 1000 appuser && \
    useradd --uid 1000 --gid appuser --shell /bin/bash --create-home appuser

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV PATH="/opt/venv/bin:$PATH"

# Copy virtual environment from builder stage
COPY --from=builder /opt/venv /opt/venv

# Set working directory and change ownership
WORKDIR /app

# Copy application code
COPY --chown=appuser:appuser app ./app

# Switch to non-root user
USER appuser

# Default port
EXPOSE 9090

# Environment variables with defaults
ENV METRICS_PORT=9090
ENV LOG_LEVEL=INFO
ENV TARGET_SERVICE_URL=""
ENV METRICS_ENDPOINT="/api/metrics/status"
ENV POLL_INTERVAL_SECONDS=2
ENV CORS_ORIGINS="*"

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "9090"]
