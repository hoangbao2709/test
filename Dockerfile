# === Stage 1: Build Frontend (Next.js) ===
FROM node:20-alpine AS frontend-build
WORKDIR /app/frontend

# Set build-time env vars để Next.js bake vào static files
ENV NEXT_PUBLIC_API_BASE=""
ENV NEXT_PUBLIC_DOGZILLA_BASE=""

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

# === Stage 3: Final Runtime (Python 3.12 + Node + Nginx + Supervisor) ===
FROM python:3.12-slim
WORKDIR /app
ENV NODE_ENV=production PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1

# Install Node.js 20, nginx, supervisor
RUN apt-get update && apt-get install -y curl nginx supervisor && \
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Copy Python packages & backend
COPY --from=backend-build /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=backend-build /usr/local/bin /usr/local/bin
COPY --from=backend-build /app/backend backend

# Copy frontend - ALL necessary files for Next.js to run
COPY --from=frontend-build /app/frontend/.next/standalone ./
COPY --from=frontend-build /app/frontend/.next/static .next/static
COPY --from=frontend-build /app/frontend/public public

# Copy nginx config
COPY nginx.conf /etc/nginx/nginx.conf

# Copy supervisord config
RUN mkdir -p /var/log/supervisor /etc/supervisor/conf.d
COPY supervisord.conf /etc/supervisor/conf.d/dogzilla.conf

# Copy entrypoint script
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

EXPOSE 80
ENTRYPOINT []
CMD ["/bin/sh", "/app/entrypoint.sh"]
