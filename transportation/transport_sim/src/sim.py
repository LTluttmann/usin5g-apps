import logging
import tb_rest_client.rest_client_ce as tbr
from datetime import datetime
import numpy as np
from dataclasses import dataclass
import time
from config import (
    USERS, 
    ROBO_ID, 
    URL, 
    NEXT_TASK_ID,
    HOME_COORD
) 
from assets import *


tenant = USERS["tenant"]
tenant_client = tbr.RestClientCE(base_url=URL)
tenant_client.login(username=tenant["mail"], password=tenant["pw"])



def get_latest_val(entity_id: str, entity_type: str = "DEVICE", key="speed", dtype=float):
    latest = tenant_client.telemetry_controller.get_latest_timeseries_using_get(
        entity_type, 
        entity_id, 
        keys=key
    )
    latest = dtype(latest[key][0]["value"])
    return latest



def update_task(task: PickingTask):
    _ = tenant_client.telemetry_controller.save_entity_telemetry_using_post(
        entity_type="ASSET",
        entity_id=NEXT_TASK_ID,
        scope="a",
        body={
            "longitude": task.coord.long,
            "latitude": task.coord.lat
        }
    )


def update_robo(robo: PickingRoboter):
    _ = tenant_client.telemetry_controller.save_entity_telemetry_using_post(
        entity_type="DEVICE",
        entity_id=ROBO_ID,
        scope="a",
        body={
            "longitude": robo.coord.long,
            "latitude": robo.coord.lat,
            "temperature": robo.temperature,
            "battery": robo.battery,
            "num_tasks": robo.num_tasks
        }
    )


def move_robo_to_task(robo: PickingRoboter, task: PickingTask):
    speed = get_latest_val(ROBO_ID, "DEVICE", "speed", float)
    robo.speed = speed
    
    diff = task.coord.numpy() - robo.coord.numpy()
    norm_diff = np.linalg.norm(diff)
    if norm_diff <= speed:
        robo.coord = task.coord
    else:
        norm = (diff / norm_diff) * speed
        robo.coord = Coordinates(*list(robo.coord.numpy() + norm))

    return robo


def load_battery(robo: PickingRoboter):
    CHARGE_RATE = 10.4
    while not robo.battery >= 100:
        robo.battery += CHARGE_RATE
        robo.battery = min(robo.battery, 100)
        update_robo(robo)
        time.sleep(1.0)
    return robo


def get_random_task(depot: Coordinates):
    add_coord = np.random.normal(0, 0.002, (2,))
    add_coord = np.clip(add_coord, -0.004, 0.004)
    new_coord = Coordinates(*list(depot.numpy() + add_coord))
    task = PickingTask(new_coord)
    return task


def main():

    depot = Coordinates(*HOME_COORD)
    robo = PickingRoboter(depot)
    task = get_random_task(depot)
    
    while True:

        if np.random.random() < 0.04:
            robo.num_tasks += 1


        if robo.num_tasks == 0:
            time.sleep(1.0)
            continue
        else:
            update_task(task)

        robo.battery -= 1.4
        robo = move_robo_to_task(robo, task)

        if robo.coord == task.coord:

            if task.coord == depot:
                robo = load_battery(robo)
            else:
                robo.num_tasks -= 1
            task = get_random_task(depot)

        if robo.battery <= 30.0:
            task = PickingTask(depot)

        update_robo(robo)
        time.sleep(1.0)


if __name__ == "__main__":
    main()