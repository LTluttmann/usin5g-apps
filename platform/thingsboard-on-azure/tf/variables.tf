variable "admin_user" {
    type = string
    sensitive = true
}

variable "admin_password" {
    type = string
    sensitive = true
}



variable "rabbitmq_user" {
    type = string
    sensitive = true
}

variable "rabbitmq_pw" {
    type = string
    sensitive = true
}



variable "psql_user" {
    type = string
    sensitive = true
}

variable "psql_pw" {
    type = string
    sensitive = true
}