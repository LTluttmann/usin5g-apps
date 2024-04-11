import logging
import tb_rest_client.rest_client_ce as tbr
from datetime import datetime
import numpy as np
from dataclasses import dataclass
import time
from typing import Union, List
from operator import itemgetter
from config import (
    USERS, 
    ROBO_ID, 
    URL, 
    NEXT_TASK_ID,
    HOME_COORD
) 
from assets import *
from utils import *
from osmr import get_route



MAX_TASKS = 10

tenant = USERS["tenant"]
tenant_client = tbr.RestClientCE(base_url=URL)
tenant_client.login(username=tenant["mail"], password=tenant["pw"])

ac = tenant_client.asset_controller
tc = tenant_client.telemetry_controller


def get_latest_val(entity_id: str, entity_type: str = "DEVICE", key="speed", dtype=float):
    latest = tc.get_latest_timeseries_using_get(
        entity_type, 
        entity_id, 
        keys=key
    )
    latest = dtype(latest[key][0]["value"])
    return latest




def update_telemetry(object: Union[PickingRoboter, PickingTask]):
    if not isinstance(object, PickingRoboter) and not isinstance(object, PickingTask):
        return
    tc.save_entity_telemetry_using_post(**object.to_dict())



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
        update_telemetry(robo)
        time.sleep(1.0)
    return robo


def get_inactive_task(ac: tbr.AssetControllerApi, tc: tbr.TelemetryControllerApi):
    tasks = get_all_tasks(ac)
    for task in tasks:
        telemetry = tc.get_latest_timeseries_using_get("ASSET", task.id.id)
        if telemetry["is_active"][0]["value"] == "false":
            return task
        
    return None
    
    #raise ValueError("no inactive tasks")


def get_random_task(depot: Coordinates, tb_task: tbr.Asset = None):
    add_coord = np.random.normal(0, 0.002, (2,))
    add_coord = np.clip(add_coord, -0.004, 0.004)
    new_coord = depot + add_coord

    # create a new thingsboard object for the task
    if tb_task is None:
        task_id = get_next_task_id(tenant_client.asset_controller)

        body = {"name": task_id, "type": "Transportauftrag"}
        # create task object
        tb_task = tenant_client.asset_controller.save_asset_using_post(body=body)

    new_task = PickingTask(new_coord.long, new_coord.lat, id=tb_task.id.id)

    # update with location information
    update_telemetry(new_task)
    return new_task



def create_robot(depot):
    old_robs = tenant_client.device_controller.get_tenant_devices_using_get(10, 0, type="robot")
    assert len(old_robs.data) == 1
    robot = old_robs.data[0]
    robot = PickingRoboter(depot, id=robot.id.id)

    update_telemetry(robot)

    return robot


def init_tasks(ac, depot):
    tasks = get_all_tasks(ac)
    task_list = []
    for task in tasks:
        new_task = get_random_task(depot, task)
        task_list.append(new_task)

    if len(task_list) < MAX_TASKS:
        tasks_to_create = MAX_TASKS - len(tasks)
        for _ in range(tasks_to_create):
            new_task = get_random_task(depot)
            task_list.append(new_task)

    return task_list



def init_state():
    # old_tasks = get_all_tasks(ac)
    # for task in old_tasks:
    #     ac.delete_asset_using_delete(task.id.id)

    depot = Coordinates(*HOME_COORD)
    robo = create_robot(depot)
    task_list = init_tasks(ac, depot)

    # # num_init_tasks = np.random.randint(1,9)
    # task_list = []
    # for _ in range(MAX_TASKS):
    #     new_task = get_random_task(depot)
    #     task_list.append(new_task)

    return depot, robo, task_list
        

def get_next_task(tasks: List[PickingTask], robo: PickingRoboter, depot) -> PickingTask:
    def get_dist(task, robo):
        diff = task.coord.numpy() - robo.coord.numpy()
        norm_diff = np.linalg.norm(diff)
        return norm_diff
    
    if len(tasks) == 0:
        return ChargingTask(depot.long, depot.lat)
    
    distances = [get_dist(task, robo) for task in tasks]
    index, _ = min(enumerate(distances), key=itemgetter(1))
    task = tasks.pop(index)

    return task


def main():
    BATTERY_DRAIN = 0.9
    depot, robo, tasks = init_state()
    task = get_next_task(tasks, robo, depot)

    task.is_next = True
    update_telemetry(task)

    route = get_route(robo.coord, task.coord)

    while True:

        if np.random.random() < 0.04 and len(tasks) < MAX_TASKS-1:
            new_task = get_inactive_task(ac, tc)
            new_task = get_random_task(depot, tb_task=new_task)
            tasks.append(new_task)

        if len(route) == 0:

            if task.coord == depot:
                robo = load_battery(robo)
            else:
                task.is_active = False
                task.is_next = False
                task.longitude = "null"
                task.latitude = "null"
                update_telemetry(task)
                # ac.delete_asset_using_delete(task.id)

            task = get_next_task(tasks, robo, depot)
            if task.coord == robo.coord:
                route = []
            else:
                route = get_route(robo.coord, task.coord)
                route_home = get_route(task.coord, depot)

            if robo.battery <= len(route + route_home.route) * BATTERY_DRAIN:
                task = ChargingTask(depot.long, depot.lat)
                route = get_route(robo.coord, task.coord)
            else:
                task.is_next = True
                update_telemetry(task)

        else:

            next_waypoint = route.pop(0)
            robo.coord = next_waypoint
            robo.battery -= BATTERY_DRAIN

        update_telemetry(robo)
        time.sleep(1.0)


if __name__ == "__main__":
    main()