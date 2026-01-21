# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""
Configuration module for the Metrics Service.

Environment variables:
    METRICS_PORT: Port to run the metrics service on (default: 9090)
    TARGET_SERVICE_URL: URL of the target service to collect metrics from (optional)
    METRICS_ENDPOINT: Endpoint path for metrics on target service (default: /api/metrics/status)
    POLL_INTERVAL_SECONDS: Interval in seconds to poll target service (default: 2)
    LOG_LEVEL: Logging level (default: INFO)
"""

import os

# Service configuration
METRICS_PORT = int(os.environ.get("METRICS_PORT", "9090"))
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO")

# Target service configuration (for active polling mode)
TARGET_SERVICE_URL = os.environ.get("TARGET_SERVICE_URL", "")
METRICS_ENDPOINT = os.environ.get("METRICS_ENDPOINT", "/api/metrics/status")
POLL_INTERVAL_SECONDS = int(os.environ.get("POLL_INTERVAL_SECONDS", "2"))

# CORS configuration
CORS_ORIGINS = os.environ.get("CORS_ORIGINS", "*").split(",")
