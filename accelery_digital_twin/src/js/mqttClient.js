// Assuming 'mqtt' is imported or available globally
let mqttClient;
let latestMessage = "Standardnachricht";

function connectToBroker() {
    const clientId = "client" + Math.random().toString(36).substring(7);
    const host = "ws://brokercredentials/ws"; // Use "wss://" for secure connections
    const options = {
        keepalive: 60,
        clientId,
        protocolId: "MQTT",
        protocolVersion: 4,
        clean: true,
        reconnectPeriod: 1000,
        connectTimeout: 30 * 1000,
        username: "brokeradmin",
        password: "brokeradmin",
    };

    mqttClient = mqtt.connect(host, options);

    mqttClient.on("connect", () => {
        console.log("Client connected:" + clientId);
        // Example: Subscribe directly upon successful connection
        subscribeToTopic("sensors");
    });

    mqttClient.on("message", (topic, message, packet) => {
        latestMessage = message.toString();
        console.log(`Received Message: ${latestMessage} On topic: ${topic}`);
    });

    mqttClient.on("error", (err) => {
        console.log("Connection Error:", err);
        mqttClient.end();
    });

    mqttClient.on("reconnect", () => {
        console.log("Reconnecting...");
    });
}

function subscribeToTopic(topic) {
    if (!mqttClient) {
        console.log("MQTT client not initialized");
        return;
    }
    mqttClient.subscribe(topic, { qos: 0 }, (error) => {
        if (error) {
            console.log("Subscribe error:", error);
        } else {
            console.log("Subscribed to:", topic);
        }
    });
}

function getLatestMessage() {
    return latestMessage;
}
connectToBroker();
export { getLatestMessage };

