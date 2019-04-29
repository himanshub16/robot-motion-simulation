#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

def callback(data):
    rospy.loginfo("LISTENER" + rospy.get_caller_id() + " I heard %s", data.data)

def worker(node_name, talk_to, listen_to):
    rospy.init_node(node_name)

    pubs = [rospy.Publisher(cname, String, queue_size=10)
            for cname in talk_to]

    for cname in listen_to:
        rospy.Subscriber(cname, String, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = '%s' % rospy.get_time()
        rospy.loginfo("TALKER" + hello_str)
        for p in pubs:
            p.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    import json
    with open('connection-graph.json') as f:
        graph = json.loads(f.read())

    from concurrent.futures import ProcessPoolExecutor, as_completed
    from multiprocessing import Process
    import time

    try:
        all_names = [each['from'] for each in graph]
        all_names.extend([each['to'] for each in graph])
        all_names = set(all_names)

        processes = []

        for node_name in all_names:
            talk_to = [each['channel_name']
                    for each in graph if each['from'] == node_name]
            listen_to = [each['channel_name']
                    for each in graph if each['to'] == node_name]

            print(node_name, talk_to, listen_to)
            p = Process(target=worker, args=(node_name, talk_to, listen_to,))
            p.start()
            processes.append(p)

        for p in processes:
            p.join()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()
        sys.exit(0)
