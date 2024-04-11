import tb_rest_client.rest_client_ce as tbr
from assets import PickingTask


__all__ = [
    "get_all_tasks",
    "get_next_task_id",
    "get_task_from_tb_object",
    "get_all_robots",
    "get_next_robot_id"
]

def get_all_tasks(ac: tbr.AssetControllerApi):
    page = 0

    tasks = []
    while True:
        res = ac.get_tenant_assets_using_get(10, page,type="Transportauftrag")
        tasks.extend(res.data)
        if res.has_next:
            page += 1
        else:
            break
            
    return tasks


def get_next_task_id(ac: tbr.AssetControllerApi, prefix="auftrag", num_digits=3):
    tasks = get_all_tasks(ac)
    if len(tasks) > 0:
        identifier = max([int(x.name.split("-")[-1]) for x in tasks]) + 1
    else:
        identifier = 1
    identifier = str(identifier).rjust(num_digits, "0")
    new_id = f"{prefix}-{identifier}"
    return new_id



def get_task_from_tb_object(tc: tbr.TelemetryControllerApi, tb_task: tbr.Asset):
    ts = tc.get_latest_timeseries_using_get("ASSET", tb_task.id.id)
    ts_dict = {k: v[0]["value"] for k,v in ts.items()}
    task = PickingTask(**ts_dict, id=tb_task.id.id)
    return task



### DEVICE UTILS

def get_all_robots(dc: tbr.DeviceControllerApi):
    page = 0

    robots = []
    while True:
        res = dc.get_tenant_devices_using_get(10, page, type="robot")
        robots.extend(res.data)
        if res.has_next:
            page += 1
        else:
            break
            
    return robots


def get_next_robot_id(dc: tbr.DeviceControllerApi, prefix="robot", num_digits=2):
    robots = get_all_robots(dc)
    if len(robots) > 0:
        identifier = max([int(x.name.split("-")[-1]) for x in robots]) + 1
    else:
        identifier = 1
    identifier = str(identifier).rjust(num_digits, "0")
    new_id = f"{prefix}-{identifier}"
    return new_id