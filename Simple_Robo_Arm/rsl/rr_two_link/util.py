#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import json
import logging
from io import StringIO
from math import pi


def get_physical_params() -> dict:
    with open('physical_parameters.json', 'r') as f:
        params = json.loads(f.read())
    return params


def get_simple_logger(name: str, verbosity=logging.WARNING, file=None) -> logging.Logger:
	logger = logging.getLogger(name)
	log_format = '%(asctime)s; %(name)s; %(levelname)s; %(message)s'
	# Create formatters and add it to handlers, add the handles to the logger
	console_handler = logging.StreamHandler()
	console_handler.setLevel(verbosity)
	console_format = logging.Formatter(log_format)
	console_handler.setFormatter(console_format)
	logger.addHandler(console_handler)
	
	# Create handlers. These say what to do with items as they get added to the logger
	if file is not None:
		# Create formatters and add it to handlers, add the handles to the logger
		file_handler = logging.FileHandler(file)
		file_handler.setLevel(verbosity)
		file_format = logging.Formatter(log_format)
		file_handler.setFormatter(file_format)
		logger.addHandler(file_handler)

	logger.setLevel(verbosity)
	logger.info(f'Log level set to: {verbosity}')
	return logger


def prettify_radians(radians:float) -> str:
	return f'{round(radians/pi, 4)}*pi rad'