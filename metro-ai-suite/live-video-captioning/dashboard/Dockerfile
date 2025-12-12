FROM python:3.13-slim
WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
RUN pip install --no-cache-dir uv==0.4.23
COPY pyproject.toml ./
COPY app.py ./
RUN uv pip install --no-cache --system .
COPY . .
EXPOSE 4173

CMD ["uv", "run", "--no-sync", "uvicorn", "app:app", "--host", "0.0.0.0", "--port", "4173"]
