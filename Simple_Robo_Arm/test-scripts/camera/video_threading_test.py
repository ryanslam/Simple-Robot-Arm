from __future__ import annotations
import sys
sys.path.append(sys.path[0] + '/../..')
import argparse
import os
from threading import Thread, local
from datetime import datetime
import asyncio
import logging
import time
from typing import Optional

import rsl.rr_two_link.camera as camera
import rsl.rr_two_link.util as util


async def run_async_generator(gen):
    start_time = time.monotonic()
    async for item in gen():
        if time.monotonic() - start_time  > 10:
            break

def run(gen):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_async_generator(gen))


if __name__ == "__main__":
    camera.logger.setLevel(logging.DEBUG)
    camera.logger.info(f'Log level changed to DEBUG')

    test_func = camera.generate_encoded_video
    # run(test_func)
    print(f'Initial benchmark complete.')
    thread = Thread(target=run, args=[test_func],)
    thread.start()

    # while 1:
    #     time.sleep(0.1)

