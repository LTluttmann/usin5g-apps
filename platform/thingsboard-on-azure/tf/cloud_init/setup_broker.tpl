#!/bin/bash
sudo apt update

# rabbit mq config
sudo apt-get -y install erlang
sudo apt-get -y install rabbitmq-server
sudo systemctl start rabbitmq-server.service
sudo systemctl enable rabbitmq-server.service
sudo rabbitmqctl add_user ${rabbitmq_user} ${rabbitmq_pw} 
sudo rabbitmqctl set_user_tags ${rabbitmq_user} administrator
sudo rabbitmqctl set_permissions -p / ${rabbitmq_user} ".*" ".*" ".*"
sudo rabbitmq-plugins enable rabbitmq_management