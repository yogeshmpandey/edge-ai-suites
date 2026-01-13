# How to Build Source

This project is primarily intended to run via Docker Compose. This page documents common build/run flows.

## Build containers

From the repository root:

```bash
docker compose build
```

## Run the stack

```bash
docker compose up
```

To force a clean rebuild:

```bash
docker compose up --build
```


Notes:
- Ensure your `.env` is configured, especially `HOST_IP`.
