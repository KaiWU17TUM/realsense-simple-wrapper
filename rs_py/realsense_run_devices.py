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

    rsw = RealsenseWrapper(arg)
    rsw.initialize()
    rsw.set_ir_laser_power(arg.rs_laser_power)
    rsw.save_calibration()
    print("Initialized RealSense devices...")

    rsw.dummy_capture(30)

    print("Starting frame capture loop...")
    try:
        c = 0
        while True:
            frames = rsw.step(display=arg.rs_display_frame)
            if not len(frames) > 0:
                print("Empty...")
                continue
            else:
                print("Running...")
            c += 1
            if c > arg.rs_fps * 10:
                break

    except:  # noqa
        print("Stopping RealSense devices...")
        rsw.stop()

    finally:
        rsw.stop()

    print("Finished")
