variable "region" {
  default = "West Europe"
}

variable "subscription_id" {
}

provider "azurerm" {
  subscription_id            = var.subscription_id
  skip_provider_registration = true
  features {
  }
}

# Shared state
terraform {
  required_version = "1.3.7"
  required_providers {
    azurerm = {
      source  = "hashicorp/azurerm"
      version = "=3.41.0"
    }
  }  
}


resource "azurerm_resource_group" "example" {
  name     = "example"
  location = var.region
}


resource "azurerm_container_group" "n8n" {
  name                = "example-n8n-aci"
  location            = azurerm_resource_group.example.location
  resource_group_name = azurerm_resource_group.example.name
  ip_address_type     = "Public"
  dns_name_label      = "example-n8n-aci"
  os_type             = "Linux"

  exposed_port {
    port     = 12345 
    protocol = "UDP"
  }

  container {
    name   = "n8n"
    image  = "n8nio/n8n"
    cpu    = "1.5"
    memory = "2"

    ports {
      port     = 5678 
      protocol = "TCP"
    }   
  }
}