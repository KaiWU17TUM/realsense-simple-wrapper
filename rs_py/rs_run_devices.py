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
        while True:
            print(f"[INFO] : step {c}")
            frames = rsw.step(display=arg.rs_display_frame)
            if not len(frames) > 0:
                print("[WARN] : Empty...")
                continue
            c += 1
            if c > arg.rs_fps * 10:
                break

    except Exception as e:
        print("[ERROR]", e)
        print("[INFO] : Stopping RealSense devices...")
        rsw.stop()

    finally:
        print("[INFO] : Stopping RealSense devices...")
        rsw.stop()

    print("[INFO] : Finished")
