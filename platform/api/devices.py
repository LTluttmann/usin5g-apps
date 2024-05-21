import tb_rest_client.rest_client_ce as tbr
from tb_rest_client.rest_client_ce import RestClientCE


def delete_all_assets(tenant_client: RestClientCE, asset_type: str):
    page = 0
    ac = tenant_client.asset_controller

    while True:
        res = ac.get_tenant_assets_using_get(10, page, type=asset_type)
                
        for tb_task in res.data:

            ac.delete_asset_using_delete(tb_task.id.id)
        
        if res.has_next:
            page += 1
        else:
            break


def get_all_devices_of_type(tenant_client: RestClientCE, device_type):
    dc = tenant_client.device_controller
    page = 0

    robots = []
    while True:
        res = dc.get_tenant_devices_using_get(10, page, type=device_type)
        robots.extend(res.data)
        if res.has_next:
            page += 1
        else:
            break
            
    return robots


def get_next_device_id(ac: tbr.DeviceControllerApi, prefix="robot", num_digits=2):
    devices = get_all_devices_of_type(ac)
    if len(devices) > 0:
        identifier = max([int(x.name.split("-")[-1]) for x in devices]) + 1
    else:
        identifier = 1
    identifier = str(identifier).rjust(num_digits, "0")
    new_id = f"{prefix}-{identifier}"
    return new_id