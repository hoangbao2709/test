from django.contrib import admin
from django.urls import path, include, re_path
from django.http import JsonResponse

def health_check(request):
    return JsonResponse({"status": "ok", "service": "dogzilla-backend"})

def debug_info(request):
    import sys
    return JsonResponse({
        "status": "ok",
        "python_version": sys.version,
        "django_started": True,
        "request_path": request.path,
        "request_method": request.method,
    })

def catch_all(request, path=''):
    return JsonResponse({
        "error": "Not found",
        "path": request.path,
        "method": request.method,
        "available_urls": [
            "/health",
            "/debug", 
            "/control/api/robots/",
            "/api/auth/",
        ]
    }, status=404)

urlpatterns = [
    path('', health_check),
    path('health', health_check),
    path('debug', debug_info),
    path('admin/', admin.site.urls),
    path('control/', include('control.urls')),
    path("api/auth/", include("authapp.urls")),
    re_path(r'^.*$', catch_all),  # Catch-all at the end
]
