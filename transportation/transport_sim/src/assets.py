from dataclasses import dataclass
import numpy as np


__all__ = [
    "Coordinates",
    "PickingRoboter",
    "ChargingTask",
    "PickingTask"
]


@dataclass
class Coordinates:
    long: float
    lat: float

    def numpy(self):
        return np.array([self.long, self.lat])
    
    def __str__(self) -> str:
        return ",".join([str(x) for x in self.numpy()])
    
    def __eq__(self, __value: object) -> bool:
        try:
            return np.all(self.numpy() == __value.numpy())
        except AttributeError:
            raise ValueError("cannot compare object of type %s with Coordinates" % type(__value))
    
    def __add__(self, __value: list):
        assert len(__value) == 2
        return Coordinates(self.long + __value[0], self.lat + __value[1])


@dataclass
class PickingRoboter:
    coord: Coordinates
    battery: float = 100
    temperature: float = 54.0
    speed: float = 0.0002
    id: str = None

    def to_dict(self):
        return {
            "entity_type": "DEVICE",
            "entity_id": self.id,
            "scope": "ts",
            "body": self.body
        }

    @property
    def body(self):
        return {
            "longitude": self.coord.long,
            "latitude": self.coord.lat,
            "temperature": self.temperature,
            "speed": self.speed,
            "battery": self.battery
        }



class Task(object):
    def __init__(self, longitude, latitude) -> None:
        self.longitude = longitude
        self.latitude = latitude
        self.coord = Coordinates(self.longitude, self.latitude)


class ChargingTask(Task):
    def __init__(self, longitude, latitude) -> None:
        super().__init__(longitude, latitude)


class PickingTask(Task):
    def __init__(self, longitude, latitude, id, is_next = False, is_active = True) -> None:
        super().__init__(longitude, latitude)
        self.id = id
        self.is_next = is_next
        self.is_active = is_active

    @property
    def body(self):
        return {
            "longitude": self.longitude,
            "latitude": self.latitude,
            "is_next": self.is_next,
            "is_active": self.is_active
        }

    def to_dict(self):
        return {
            "entity_type": "ASSET",
            "entity_id": self.id,
            "scope": "ts",
            "body": self.body
        }
