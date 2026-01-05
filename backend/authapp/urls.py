from django.urls import path
from .views import login_view, register_view, save_user_url, link_robot, me_view

urlpatterns = [
    path("login/", login_view, name="login"),
    path("register/", register_view, name="register"),
    path("save-url/", save_user_url, name="save_user_url"),
    path("link-robot/", link_robot, name="link_robot"),
    path("me/", me_view, name="me"),
]
