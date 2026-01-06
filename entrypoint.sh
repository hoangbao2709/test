#!/bin/sh
set -e

# Get PORT from environment or default to 80
PORT=${PORT:-80}

echo "=== Dogzilla Backend Starting ==="
echo "PORT: $PORT"
echo "Railway assigned port: ${PORT}"

# Update nginx to listen on the specified PORT
sed -i "s/listen 80/listen $PORT/g" /etc/nginx/nginx.conf
sed -i "s/listen \[::\]:80/listen [::]:$PORT/g" /etc/nginx/nginx.conf

# Run Django migrations
echo "Running Django migrations..."
cd /app/backend
python manage.py migrate --noinput

echo "Starting supervisord..."
echo "Backend will run on: 127.0.0.1:8000"
echo "Frontend will run on: 127.0.0.1:3000"
echo "Nginx will listen on: $PORT"

# Start supervisord
exec /usr/bin/supervisord -c /etc/supervisor/conf.d/dogzilla.conf
