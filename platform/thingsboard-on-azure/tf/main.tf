resource "random_pet" "rg_name" {
  prefix = "usin5g-group"
}

resource "azurerm_resource_group" "rg" {
  name     = random_pet.rg_name.id
  location = "germanywestcentral"
}

resource "random_integer" "rd_num" {
  min = 1
  max = 50000
}


# Create virtual network
resource "azurerm_virtual_network" "vnet" {
  name                = "usin5g-tb-vnet-${random_integer.rd_num.result}"
  address_space       = ["10.0.0.0/16"]
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name
}



# -----------------------------------------------------------------------
# ------------------------- POSTGRES SERVER -----------------------------
# -----------------------------------------------------------------------

resource "azurerm_network_security_group" "default" {
  name                = "postgres-dns-nsg-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name

  security_rule {
    name                       = "test123"
    priority                   = 100
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_range     = "*"
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
}



# Create subnet
resource "azurerm_subnet" "postgres_subnet" {
  name                 = "usin5g-postgres-subnet-${random_integer.rd_num.result}"
  resource_group_name  = azurerm_resource_group.rg.name
  virtual_network_name = azurerm_virtual_network.vnet.name
  address_prefixes     = ["10.0.2.0/24"]
  service_endpoints    = ["Microsoft.Storage"]

  delegation {
    name = "fs"

    service_delegation {
      name = "Microsoft.DBforPostgreSQL/flexibleServers"

      actions = [
        "Microsoft.Network/virtualNetworks/subnets/join/action",
      ]
    }
  }
}

resource "azurerm_subnet_network_security_group_association" "default" {
  subnet_id                 = azurerm_subnet.postgres_subnet.id
  network_security_group_id = azurerm_network_security_group.default.id
}

resource "azurerm_private_dns_zone" "default" {
  name                = "usin5g.postgres.database.azure.com"
  resource_group_name = azurerm_resource_group.rg.name

  depends_on = [azurerm_subnet_network_security_group_association.default]
}

resource "azurerm_private_dns_zone_virtual_network_link" "default" {
  name                  = "usin5g-pdzvnetlink-${random_integer.rd_num.result}.com"
  private_dns_zone_name = azurerm_private_dns_zone.default.name
  virtual_network_id    = azurerm_virtual_network.vnet.id
  resource_group_name   = azurerm_resource_group.rg.name
}


resource "azurerm_postgresql_flexible_server" "default" {
  name                   = "usin5g-database-server-${random_integer.rd_num.result}"
  resource_group_name    = azurerm_resource_group.rg.name
  location               = azurerm_resource_group.rg.location
  version                = "12" # postgres version to use
  delegated_subnet_id    = azurerm_subnet.postgres_subnet.id
  private_dns_zone_id    = azurerm_private_dns_zone.default.id
  administrator_login    = var.psql_user
  administrator_password = var.psql_pw
  storage_mb             = 32768
  sku_name               = "B_Standard_B2s"
  backup_retention_days  = 7
  depends_on = [azurerm_private_dns_zone_virtual_network_link.default]
}


# -----------------------------------------------------------------------
# ------------------------- THINGSBOARD SETUP ---------------------------
# -----------------------------------------------------------------------

# Create subnet
resource "azurerm_subnet" "subnet" {
  name                 = "thingsboard-subnet-${random_integer.rd_num.result}"
  resource_group_name  = azurerm_resource_group.rg.name
  virtual_network_name = azurerm_virtual_network.vnet.name
  address_prefixes     = ["10.0.1.0/24"]
}



# Create public IPs
resource "azurerm_public_ip" "thingsboard_public_ip" {
  name                = "usin5g-tb-ip-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name
  allocation_method   = "Static"
}

resource "azurerm_public_ip" "broker_public_ip" {
  name                = "usin5g-broker-ip-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name
  allocation_method   = "Static"
}


# Create Network Security Group and rule
resource "azurerm_network_security_group" "thingsboard_nsg" {
  name                = "usin5g-thingsboard-security-group-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name
  security_rule {
    name                       = "SSH"
    priority                   = 300
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_range     = "22"
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
  security_rule {
    name                       = "HTTP"
    priority                   = 320
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_ranges    = [80, 8080, 5000]
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
  security_rule {
    name                       = "MQTT"
    priority                   = 330
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_ranges    = [1883, 5672, 15672]
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
  security_rule {
    name                       = "HTTPS"
    priority                   = 340
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_range     = "443"
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
  security_rule {
    name                       = "SMTP"
    priority                   = 350
    direction                  = "Inbound"
    access                     = "Allow"
    protocol                   = "Tcp"
    source_port_range          = "*"
    destination_port_range     = "443"
    source_address_prefix      = "*"
    destination_address_prefix = "*"
  }
}

# Create network interface
resource "azurerm_network_interface" "thingsboard_nic" {
  name                = "usin5g-tb-nic-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name

  ip_configuration {
    name                          = "nic-configuration-thingsboard-${random_integer.rd_num.result}"
    subnet_id                     = azurerm_subnet.subnet.id
    private_ip_address_allocation = "Dynamic"
    public_ip_address_id          = azurerm_public_ip.thingsboard_public_ip.id
  }
}

resource "azurerm_network_interface" "broker_nic" {
  name                = "usin5g-broker-nic-${random_integer.rd_num.result}"
  location            = azurerm_resource_group.rg.location
  resource_group_name = azurerm_resource_group.rg.name

  ip_configuration {
    name                          = "nic-configuration-broker-${random_integer.rd_num.result}"
    subnet_id                     = azurerm_subnet.subnet.id
    private_ip_address_allocation = "Dynamic"
    public_ip_address_id          = azurerm_public_ip.broker_public_ip.id
  }
}

# Connect the security group to the network interface
resource "azurerm_network_interface_security_group_association" "tb_nic_nsg" {
  network_interface_id      = azurerm_network_interface.thingsboard_nic.id
  network_security_group_id = azurerm_network_security_group.thingsboard_nsg.id
}

resource "azurerm_network_interface_security_group_association" "mq_nic_nsg" {
  network_interface_id      = azurerm_network_interface.broker_nic.id
  network_security_group_id = azurerm_network_security_group.thingsboard_nsg.id
}


# Create (and display) an SSH key
resource "tls_private_key" "example_ssh" {
  algorithm = "RSA"
  rsa_bits  = 4096
}


locals {

    rabbitmq_init = templatefile(
    "${path.module}/cloud_init/setup_broker.tpl",
    {
      rabbitmq_user        = var.rabbitmq_user    
      rabbitmq_pw          = var.rabbitmq_pw
    }
  )
}


# Create virtual machine
resource "azurerm_linux_virtual_machine" "rabbitmq" {
  name                  = "usin5g-rabbitmq-${random_integer.rd_num.result}"
  location              = azurerm_resource_group.rg.location
  resource_group_name   = azurerm_resource_group.rg.name
  network_interface_ids = [azurerm_network_interface.broker_nic.id]
  size                  = "Standard_B1s"

  os_disk {
    name                 = "rabbit-osdisk-${random_integer.rd_num.result}"
    caching              = "ReadWrite"
    storage_account_type = "Premium_LRS"
  }

  source_image_reference {
    publisher = "Canonical"
    offer     = "0001-com-ubuntu-server-jammy"
    sku       = "22_04-lts-gen2"
    version   = "latest"
  }

  computer_name          = "messagebroker"  
  admin_username         = var.admin_user
  admin_password         = var.admin_password
  custom_data            = "${base64encode(local.rabbitmq_init)}"

  disable_password_authentication = false
}



locals {
    thingsboard_init = templatefile(
    "${path.module}/cloud_init/setup_thingsboard.tpl",
    {
      postgres_server_name = azurerm_postgresql_flexible_server.default.name
      rabbit_host          = azurerm_linux_virtual_machine.rabbitmq.private_ip_address
      server_dns_or_ip     = azurerm_public_ip.thingsboard_public_ip.ip_address
      postgres_user        = var.psql_user
      postgres_pw          = var.psql_pw
      rabbitmq_user        = var.rabbitmq_user    
      rabbitmq_pw          = var.rabbitmq_pw
    }
  )

  depends_on = [azurerm_postgresql_flexible_server.default, azurerm_linux_virtual_machine.rabbitmq]

}



# Create virtual machine
resource "azurerm_linux_virtual_machine" "thingsboard" {
  name                  = "usin5g-thingsboard-${random_integer.rd_num.result}"
  location              = azurerm_resource_group.rg.location
  resource_group_name   = azurerm_resource_group.rg.name
  network_interface_ids = [azurerm_network_interface.thingsboard_nic.id]
  size                  = "Standard_B2s"

  os_disk {
    name                 = "thingsboard-osdisk-${random_integer.rd_num.result}"
    caching              = "ReadWrite"
    storage_account_type = "Premium_LRS"
  }

  source_image_reference {
    publisher = "Canonical"
    offer     = "0001-com-ubuntu-server-jammy"
    sku       = "22_04-lts-gen2"
    version   = "latest"
  }

  admin_ssh_key {
    username   = "thingsboard2023"
    public_key = tls_private_key.example_ssh.public_key_openssh
  }

  computer_name                   = "thingsboardonazure"
  admin_username                  = var.admin_user
  admin_password                  = var.admin_password

  custom_data = "${base64encode(local.thingsboard_init)}"


  disable_password_authentication = false

  depends_on = [azurerm_postgresql_flexible_server.default, azurerm_linux_virtual_machine.rabbitmq]
}
