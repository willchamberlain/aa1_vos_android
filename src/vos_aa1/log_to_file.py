# log ROS topics to files
# - the ROS logging config pages don't work well so do it programmatically in Python
# - see https://docs.python.org/2/library/logging.html#module-logging  ,  https://docs.python.org/2/howto/logging.html
# https://docs.python.org/2.3/lib/node304.html
# https://docs.python.org/2/howto/logging-cookbook.html

import logging

import rospy

def start_logger():
    logger = logging.getLogger('file_logger')
    handler = logging.handlers.RotatingFileHandler('/mnt/nixbig/ownCloud/project_AA1__1_1/results/111_logs/logit.log','a',50000000,40)
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)



if __name__ == "__main__":
    rospy.init_node('detect_feature_server')

    start_logger()
