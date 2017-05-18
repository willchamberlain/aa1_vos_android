#!/usr/bin/env python

import logging
import logging.handlers
from start_logger import start_logger

if __name__ == '__main__':

    logger = start_logger('test_logger','/tmp/test_log.log')

    # Log some messages
    for i in range(20):
        logger.info('i = %d' % i)
