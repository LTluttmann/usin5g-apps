import tb_rest_client.rest_client_ce as tbr
from tb_rest_client.rest_client_ce import RestClientCE

from .customers import get_id_of_customer_w_name
from .utils import maybe_cast_str_to_num


def get_devices_of_customer(tenant_client: RestClientCE, customer: str):
    customer_id = get_id_of_customer_w_name(customer)
    customer_devices = tenant_client.device_controller.get_customer_devices_using_get(
        customer_id, 10, 0
    ).data
    return customer_devices


def get_latest_telemetry_of_device(client: RestClientCE, device_id: str):
    controller = client.telemetry_controller
    telemetry = controller.get_latest_timeseries_using_get(
        "DEVICE", device_id
    )
    if len(telemetry) == 0:
        return None
    kvs = {
        k: maybe_cast_str_to_num(v[-1]["value"]) 
        for k, v in telemetry.items()
    }
    return kvs