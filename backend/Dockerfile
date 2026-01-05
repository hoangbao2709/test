FROM python:3.12-slim

# Tắt tải bytecode + buffer output
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /app

# Cài dependencies hệ thống (postgre, build tools nếu cần)
RUN apt-get update && apt-get install -y \
    build-essential \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy file requirements
COPY requirements.txt .

# Cài lib Python
RUN pip install --no-cache-dir -r requirements.txt

# Copy toàn bộ project backend
COPY . .

# Thu gom static files
RUN python manage.py collectstatic --noinput

# Expose port gunicorn
EXPOSE 8000

# Chạy bằng Gunicorn
CMD ["daphne", "-b", "0.0.0.0", "-p", "8000", "backend.asgi:application"]

