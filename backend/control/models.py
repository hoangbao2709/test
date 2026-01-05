from django.db import models

class Robot(models.Model):
    id = models.CharField(primary_key=True, max_length=64) 
    name = models.CharField(max_length=128, default="Robot A")
    addr = models.CharField(max_length=256, blank=True, default="")  

    location_lat = models.FloatField(null=True, blank=True)
    location_lon = models.FloatField(null=True, blank=True)
    cleaning_progress = models.FloatField(default=0)
    floor = models.CharField(max_length=16, default="1st")
    status_text = models.CharField(max_length=64, default="Resting")
    water_level = models.IntegerField(default=50)
    battery = models.IntegerField(default=85)
    fps = models.IntegerField(default=30)

    def __str__(self): return self.name
