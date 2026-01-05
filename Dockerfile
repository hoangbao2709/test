# === Stage 1: Build Frontend (Next.js) ===
FROM node:20-alpine AS frontend-build
WORKDIR /app/frontend
COPY frontend/package*.json ./
RUN npm install --frozen-lockfile
COPY frontend .
RUN npm run build

# === Stage 2: Build Backend (Django) ===
FROM python:3.12-slim AS backend-build
WORKDIR /app/backend
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y build-essential libpq-dev && rm -rf /var/lib/apt/lists/*
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY backend .
RUN python manage.py collectstatic --noinput 2>/dev/null || true

# === Stage 3: Final Runtime (Python + Node + Supervisord) ===
FROM node:20-slim
WORKDIR /app
ENV NODE_ENV=production PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1

# Install Python, supervisord, and build tools
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-dev supervisor curl build-essential && \
    rm -rf /var/lib/apt/lists/*

# Copy Python packages & backend
COPY --from=backend-build /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=backend-build /usr/local/bin /usr/local/bin
COPY --from=backend-build /app/backend backend

# Copy frontend (built app)
COPY frontend/package*.json frontend/
COPY --from=frontend-build /app/frontend/.next frontend/.next
COPY --from=frontend-build /app/frontend/public frontend/public
COPY --from=frontend-build /app/frontend/next.config.ts frontend/ 2>/dev/null || true
COPY --from=frontend-build /app/frontend/tsconfig.json frontend/ 2>/dev/null || true
RUN cd frontend && npm install --production

# Create supervisord config
RUN mkdir -p /var/log/supervisor && cat > /etc/supervisor/conf.d/dogzilla.conf << 'EOF'
[supervisord]
nodaemon=true
logfile=/var/log/supervisor/supervisord.log

[program:backend]
directory=/app/backend
command=python3 -m daphne -b 0.0.0.0 -p 8000 backend.asgi:application
autostart=true
autorestart=true
stderr_logfile=/var/log/supervisor/backend.err.log
stdout_logfile=/var/log/supervisor/backend.out.log

[program:frontend]
directory=/app/frontend
command=npm start
autostart=true
autorestart=true
stderr_logfile=/var/log/supervisor/frontend.err.log
stdout_logfile=/var/log/supervisor/frontend.out.log
EOF

EXPOSE 3000 8000
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/dogzilla.conf"]
