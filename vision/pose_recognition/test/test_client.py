import websockets
import rospy
from std_msgs.msg import String
import asyncio


async def websocket_handler(pub):
    uri = "ws://20.52.7.116:1234"  # Replace with your WebSocket server address
    async with websockets.connect(uri) as websocket:
        while not rospy.is_shutdown():
            try:
                message = await websocket.recv()
                rospy.loginfo("Received message from server: %s", message)

                # Publish the received message to a ROS topic
                pub.publish(String(message))  # Change the message type as needed

            except websockets.exceptions.ConnectionClosed:
                rospy.logwarn("WebSocket connection closed.")
                break



if __name__ == '__main__':
    rospy.init_node('websocket_client_node')
    pub = rospy.Publisher('websocket_data', String, queue_size=10)  # Change topic name and message type as needed

    loop = asyncio.get_event_loop()
    loop.run_until_complete(websocket_handler(pub))

    rospy.spin()
