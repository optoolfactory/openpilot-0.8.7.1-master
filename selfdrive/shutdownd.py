#!/usr/bin/env python3

import os
import time
import cereal.messaging as messaging
from selfdrive.ntune import ntune_scc_get

def main():
  # 60초가 지나면 자동 종료된다.
  shutdown_at = 12 * 10
  shutdown_count = 0
  device_state_sock = messaging.sub_sock('deviceState')

  while 1:
    msg = messaging.recv_sock(device_state_sock, wait=True)
    if msg is not None:
      if not msg.deviceState.started and not msg.deviceState.usbOnline:
        shutdown_count += 5
      else:
        shutdown_count = 0

      print('distanceGap', ntune_scc_get('distanceGap'))
      print('accelProfile', ntune_scc_get('accelProfile'))

      #print('current', shutdown_count, 'shutdown_at', shutdown_at)

      if shutdown_count >= shutdown_at > 0:
        os.system('LD_LIBRARY_PATH="" svc power shutdown')

    time.sleep(5)


if __name__ == "__main__":
  main()