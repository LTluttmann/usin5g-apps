
import requests
from typing import List, Union
from  assets import Coordinates


API_KEY = "XXX"
ROUTE_TYPE = "foot-walking"
root_url = "https://api.openrouteservice.org/v2/directions/{route_type}?api_key={key}&start={start}&end={end}"



class Route(object):
    def __init__(self, coordinates: List[Union[list, Coordinates]]) -> None:
        self.route = [
            waypoint 
            if isinstance(waypoint, Coordinates) 
            else Coordinates(*waypoint) 
            for waypoint in coordinates
        ]

    def __getitem__(self,key):
        return self.route[key]
    
    def __len__(self):
        return self.route.__len__()
    
    def pop(self, index=-1) :
        return self.route.pop(index)
    
    def __add__(self, other):
        return Route(self.route + other)


def get_route(start: Coordinates, end: Coordinates):



    url = root_url.format(route_type=ROUTE_TYPE, key=API_KEY, start=str(start), end=str(end))

    response = requests.get(url)

    # Parse the response to get routing information
    data = response.json()

    coordinates = data["features"][0]["geometry"]["coordinates"]
    
    coordinates.append(end)
    route = Route(coordinates)
    return route

start = Coordinates(*[8.681495, 49.41461])
end = Coordinates(*[8.687872,49.420318])

if __name__ == "__main__":

    get_route(start, end)