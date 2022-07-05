import time

from rs_py import printout
from rs_py import get_rs_parser
from rs_py import RealsenseWrapper


if __name__ == "__main__":
    arg = get_rs_parser().parse_args()
    print("========================================")
    print(">>>>> args <<<<<")
    print("========================================")
    for k, v in vars(arg).items():
        print(f"{k} : {v}")
    print("========================================")

    rsw = RealsenseWrapper(arg, arg.rs_dev)
    rsw.initialize()
    rsw.set_ir_laser_power(arg.rs_laser_power)
    rsw.save_calibration()

    rsw.dummy_capture(arg.rs_fps * 5)

    try:
        c = 0
        max_c = int(1e8)
        while True:
            printout(f"Step {c:8d}", 'i')
            frames = rsw.step(
                display=arg.rs_display_frame,
                display_and_save_with_key=arg.rs_save_with_key
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
