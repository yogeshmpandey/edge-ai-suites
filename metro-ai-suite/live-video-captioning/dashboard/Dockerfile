FROM python:3.13-slim
WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

RUN pip install --no-cache-dir uv==0.4.23
COPY pyproject.toml ./
COPY src/ ./src/
RUN uv pip install --no-cache --system .
EXPOSE 4173

CMD ["python", "-m", "src.app"]
