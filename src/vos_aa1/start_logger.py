#!/usr/bin/env python

import logging
import logging.handlers

def start_logger(logger_name_, log_file_):
    logger = logging.getLogger(logger_name_)
    logger.setLevel(logging.INFO)
    handler = logging.handlers.RotatingFileHandler(log_file_,maxBytes=500000, backupCount=50)
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger
