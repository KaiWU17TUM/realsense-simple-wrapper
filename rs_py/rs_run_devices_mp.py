import multiprocessing as mp
import os
import threading
import time

from rs_py import rs

from rs_py.utility import printout
from rs_py.wrapper import get_rs_parser
from rs_py.wrapper import RealsenseWrapper

ctx = None


def thread(dev_sn, arg):
    print("Thread pid:", os.getpid(),
          " tid:", threading.current_thread().ident)

    global ctx
    if ctx is None:
        print(ctx)
        ctx = rs.context()

    rsw = RealsenseWrapper(arg, dev_sn, ctx)
    rsw.initialize()
    rsw.set_ir_laser_power(arg.rs_laser_power)
    rsw.save_calib()

    rsw.flush_frames(arg.rs_fps * 5)

    try:
        c = 0
        max_c = int(1e8)
        while True:
            printout(f"Step {c:8d}", 'i')
            frames = rsw.step(
                display=0,
                display_and_save_with_key=False
            )
            if not len(frames) > 0:
                printout(f"Empty...", 'w')
                continue
            c += 1
            if c > arg.rs_fps * arg.rs_steps or c > max_c:
                break

    except Exception as e:
        printout(f"{e}", 'e')
        printout(f"Stopping RealSense devices...", 'i')
        rsw.stop()

    finally:
        printout(f"Final RealSense devices...", 'i')
        rsw.stop()

    printout(f"Finished...", 'i')


def main():

    arg = get_rs_parser().parse_args()
    print("========================================")
    print(">>>>> args <<<<<")
    print("========================================")
    for k, v in vars(arg).items():
        print(f"{k} : {v}")
    print("========================================")

    print("TestMultiProcess pid:", os.getpid(),
          " tid:", threading.current_thread().ident)
    p1 = mp.Process(target=thread, args=('001622070408', arg,))
    time.sleep(3)
    p2 = mp.Process(target=thread, args=('001622070717', arg,))
    p3 = mp.Process(target=thread, args=('001622071039', arg,))
    p1.start()
    p2.start()
    p3.start()
    p1.join()
    p2.join()
    p3.join()


if __name__ == '__main__':
    print("mainThread pid:", os.getpid(),
          " tid:", threading.current_thread().ident)
    main()
