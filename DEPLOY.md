# Dogzilla Robot - Full Stack Deploy

## Kiến trúc
Dự án chạy trong 1 Docker container duy nhất với 3 services:
- **Backend** (Django + Daphne): Port 8000
- **Frontend** (Next.js): Port 3000  
- **Nginx**: Port 80 (entry point, proxy requests)

```
Railway (Port 80)
    ↓
  Nginx
    ├── /api/* → Backend (Django)
    ├── /admin/* → Backend
    ├── /control/* → Backend
    └── /* → Frontend (Next.js)
```

## Deploy lên Railway

### 1. Push code lên GitHub
```bash
git add .
git commit -m "Setup full-stack Docker deployment"
git push origin main
```

### 2. Tạo service mới trên Railway
1. Đăng nhập [Railway](https://railway.app)
2. New Project → Deploy from GitHub repo
3. Chọn repository này
4. Railway sẽ tự động detect `railway.toml` và build

### 3. Cấu hình biến môi trường
Trong Railway dashboard → Variables, thêm các biến sau:

**Backend (Django):**
```env
DEBUG=False
SECRET_KEY=your-secret-key-here
ALLOWED_HOSTS=${{RAILWAY_PUBLIC_DOMAIN}}
DATABASE_ENGINE=django.db.backends.sqlite3
DATABASE_NAME=/app/backend/db.sqlite3
MONGO_URI=your-mongo-uri
MONGO_DB_NAME=robot_users
MONGO_USERS_COLLECTION=users
ROBOT_REG_SECRET=your-robot-secret
```

**Frontend:**
```env
NEXT_PUBLIC_API_BASE=
NEXT_PUBLIC_DOGZILLA_BASE=
```
> **Lưu ý:** Để trống `NEXT_PUBLIC_API_BASE` để frontend dùng relative path (cùng domain qua nginx)

### 4. Expose port 80
Railway sẽ tự động expose port 80 (định nghĩa trong Dockerfile)

### 5. Truy cập
Sau khi deploy thành công, truy cập:
- Frontend: `https://your-app.railway.app/`
- Backend API: `https://your-app.railway.app/api/`
- Admin: `https://your-app.railway.app/admin/`

## Local Development với Docker

```bash
# Build image
docker build -t dogzilla-fullstack .

# Run container
docker run -p 80:80 \
  -e DEBUG=True \
  -e SECRET_KEY=dev-secret-key \
  -e ALLOWED_HOSTS=localhost \
  dogzilla-fullstack

# Truy cập
# Frontend: http://localhost/
# Backend: http://localhost/api/
```

## Cấu trúc file quan trọng

- `Dockerfile`: Multi-stage build (frontend + backend + nginx)
- `nginx.conf`: Reverse proxy config
- `railway.toml`: Railway deployment config
- `.dockerignore`: Loại bỏ file không cần thiết khi build
- `frontend/.env.production`: Biến môi trường production cho Next.js

## Troubleshooting

### Frontend không kết nối được backend
- Kiểm tra nginx logs: `docker logs <container> | grep nginx`
- Đảm bảo `NEXT_PUBLIC_API_BASE` để trống hoặc không set

### Backend không start
- Kiểm tra Django logs: `docker logs <container> | grep backend`
- Kiểm tra database connection và migrations

### Port 80 không accessible
- Railway tự động map port, không cần cấu hình thêm
- Kiểm tra Nginx có đang listen port 80

## Logs trên Railway
Railway dashboard → Deployments → Click vào deployment → View logs
Hoặc dùng Railway CLI:
```bash
railway logs
```
