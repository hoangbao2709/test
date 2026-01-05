#!/bin/sh
set -e

# Get PORT from environment or default to 80
PORT=${PORT:-80}

echo "Starting services on port $PORT..."

# Update nginx to listen on the specified PORT
sed -i "s/listen 80/listen $PORT/g" /etc/nginx/nginx.conf
sed -i "s/listen \[::\]:80/listen [::]:$PORT/g" /etc/nginx/nginx.conf

# Start supervisord
exec /usr/bin/supervisord -c /etc/supervisor/conf.d/dogzilla.conf
