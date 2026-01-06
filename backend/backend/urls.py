from django.contrib import admin
from django.urls import path, include
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

urlpatterns = [
    path('', health_check),
    path('health', health_check),
    path('debug', debug_info),
    path('admin/', admin.site.urls),
    path('control/', include('control.urls')),
    path("api/auth/", include("authapp.urls")),
]
