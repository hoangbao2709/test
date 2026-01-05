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

# === Stage 3: Final Runtime (Python + Node + Nginx + Supervisor) ===
FROM node:20-slim
WORKDIR /app
ENV NODE_ENV=production PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1

# Install Python, nginx, supervisor
RUN apt-get update && apt-get install -y \
    python3 python3-pip nginx supervisor curl && \
    rm -rf /var/lib/apt/lists/*

# Copy Python packages & backend
COPY --from=backend-build /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=backend-build /usr/local/bin /usr/local/bin
COPY --from=backend-build /app/backend backend

# Copy frontend - ALL necessary files for Next.js to run
COPY --from=frontend-build /app/frontend/.next/standalone ./
COPY --from=frontend-build /app/frontend/.next/static frontend/.next/static
COPY --from=frontend-build /app/frontend/public frontend/public

# Copy nginx config
COPY nginx.conf /etc/nginx/nginx.conf

# Create supervisord config to run all services
RUN mkdir -p /var/log/supervisor && cat > /etc/supervisor/conf.d/dogzilla.conf << 'EOF'
[supervisord]
nodaemon=true
logfile=/var/log/supervisor/supervisord.log

[program:backend]
directory=/app/backend
command=python3 -m daphne -b 127.0.0.1 -p 8000 backend.asgi:application
autostart=true
autorestart=true
stderr_logfile=/var/log/supervisor/backend.err.log
stdout_logfile=/var/log/supervisor/backend.out.log

[program:frontend]
directory=/app/frontend
command=node server.js
autostart=true
autorestart=true
environment=PORT=3000,HOSTNAME="0.0.0.0"
stderr_logfile=/var/log/supervisor/frontend.err.log
stdout_logfile=/var/log/supervisor/frontend.out.log

[program:nginx]
command=/usr/sbin/nginx -g "daemon off;"
autostart=true
autorestart=true
stderr_logfile=/var/log/supervisor/nginx.err.log
stdout_logfile=/var/log/supervisor/nginx.out.log
EOF

EXPOSE 80
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/dogzilla.conf"]
