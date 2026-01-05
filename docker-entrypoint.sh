#!/bin/sh
set -e

echo "=== Starting Dogzilla (FE + BE) ==="

# Start Django backend on port 8000
echo "[Backend] Starting Django/Daphne on port 8000..."
cd /app/backend
python -m pip install -q daphne 2>/dev/null || true
python manage.py migrate --noinput 2>/dev/null || true
python -m daphne -b 0.0.0.0 -p 8000 backend.asgi:application > /tmp/backend.log 2>&1 &
BACKEND_PID=$!
echo "[Backend] Started with PID $BACKEND_PID"
sleep 2

# Start Next.js frontend on port 3000
echo "[Frontend] Starting Next.js on port 3000..."
cd /app/frontend
npm start > /tmp/frontend.log 2>&1 &
FRONTEND_PID=$!
echo "[Frontend] Started with PID $FRONTEND_PID"

# Keep container running
wait
FRONTEND_PID=$!
echo "[Frontend] Started with PID $FRONTEND_PID"

# Trap signals
trap 'kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit' TERM INT

# Wait for processes
wait
