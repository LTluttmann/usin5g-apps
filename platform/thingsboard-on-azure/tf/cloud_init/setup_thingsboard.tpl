#!/bin/bash
sudo apt update

# --------------- setup java ---------------
sudo apt --fix-broken -y install

sudo apt -y install openjdk-11-jdk

sudo update-alternatives --config java


# --------------- setup thingsboard ---------------
wget https://github.com/thingsboard/thingsboard/releases/download/v3.6.2/thingsboard-3.6.2.deb
sudo dpkg -i thingsboard-3.6.2.deb

# config
# DB Configuration 
sudo tee -a /etc/thingsboard/conf/thingsboard.conf > /dev/null <<EOT
export DATABASE_TS_TYPE=sql
export SPRING_DATASOURCE_URL=jdbc:postgresql://${postgres_server_name}.postgres.database.azure.com:5432/thingsboard
export SPRING_DATASOURCE_USERNAME=${postgres_user}
export SPRING_DATASOURCE_PASSWORD=${postgres_pw}
# Specify partitioning size for timestamp key-value storage. Allowed values: DAYS, MONTHS, YEARS, INDEFINITE.
export SQL_POSTGRES_TS_KV_PARTITIONING=MONTHS
EOT

sudo tee -a /etc/thingsboard/conf/thingsboard.conf > /dev/null <<EOT
export TB_QUEUE_TYPE=rabbitmq
export TB_QUEUE_RABBIT_MQ_USERNAME=${rabbitmq_user}
export TB_QUEUE_RABBIT_MQ_PASSWORD=${rabbitmq_pw}
export TB_QUEUE_RABBIT_MQ_HOST=${rabbit_host}
export TB_QUEUE_RABBIT_MQ_PORT=5672
EOT

sudo /usr/share/thingsboard/bin/install/install.sh
sudo service thingsboard start

sudo apt install nginx

# Using Here Document
sudo cat <<EOL > "/etc/nginx/sites-available/thingsboard"
server {
    listen 80;
    server_name ${server_dns_or_ip};

    location / {
        proxy_pass http://localhost:8080;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    error_page 500 502 503 504 /50x.html;
    location = /50x.html {
        root /usr/share/nginx/html;
    }
}
EOL

sudo ln -s /etc/nginx/sites-available/thingsboard /etc/nginx/sites-enabled

# Run sudo nginx -t
sudo nginx -t

# Check the exit status
if [ $? -eq 0 ]; then
    echo "Nginx configuration test successful."
    # Additional actions if the test is successful
    sudo systemctl restart nginx
else
    echo "Nginx configuration test failed."
    # Additional actions if the test fails
fi