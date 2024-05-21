import tb_rest_client.rest_client_ce as tbr
from tb_rest_client.rest_client_ce import RestClientCE


def get_all_customer_names(tenant_client: RestClientCE):
    customers = tenant_client.customer_controller.get_customers_using_get(10,0).data
    return [x.title for x in customers]

def get_id_of_customer_w_name(tenant_client: RestClientCE, name: str):
    customers = tenant_client.customer_controller.get_customers_using_get(10,0).data
    for customer in customers:
        if customer.name == name:
            return customer.id.id
    raise ValueError("No customer with name %s found" % name)