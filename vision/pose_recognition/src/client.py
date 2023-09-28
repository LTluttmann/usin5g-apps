#!/usr/bin/env python

import asyncio
import websockets
import logging
import rospy
from std_srvs.srv import SetBool
import argparse


logging.basicConfig(level=logging.DEBUG)
    


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ws_port', type=str, default="1234")
    parser.add_argument('--server_ip', type=str, default="localhost")
    args = parser.parse_args()
    return args



class LidNode(object):
    """implement ROS logic here"""
    def __init__(self) -> None:

        rospy.init_node("websocket_ctrl")
        rospy.loginfo("Starting websocket_ctrl node.")

        self.last_msg = None

    def lid_ctrl(cmd):
        """

        """
        rospy.wait_for_service("lid_ctrl")
        try:
            srv_lid = rospy.ServiceProxy("lid_ctrl", SetBool)
            resp = srv_lid(cmd)
            rospy.loginfo(resp.message)
            return resp.success
        except rospy.ServiceException as e:
            print(e)

    def publish(self, msg):
        
        if msg != self.last_msg:
            self.last_msg = msg
            self.lid_ctrl(msg)
            


async def main(lid_controller: LidNode, args):

    retry_counts = 0

    while True:
        try:
            async with websockets.connect("ws://{}:{}".format(args.server_ip, args.ws_port)) as socket:
                retry_counts = 0
                while not rospy.is_shutdown():
                    logging.info("awaiting event message from server")
                    response = await socket.recv()
                    logging.info("received event message from server")
                    if response in ["up", "down"]:
                        lid_controller.publish(response)


        except Exception as e:
            print(e.__str__)
            print("Connection closed. Reconnecting... (%s more attempts)" % (3 - retry_counts))
            retry_counts += 1


        finally:
            if retry_counts > 3:
                print("Connection retries exeeded. Aborting...")
                break

        await asyncio.sleep(30)  # Adjust the reconnection interval as needed


if __name__ == "__main__":
    args = get_args()
    lid_controller = LidNode()
    asyncio.get_event_loop().run_until_complete(main(lid_controller, args))
    rospy.spin()

