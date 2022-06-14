from rs_py import get_rs_parser
from rs_py import initialize_rs_devices


if __name__ == "__main__":
    arg = get_rs_parser().parse_args()
    print("========================================")
    for k, v in vars(arg).items():
        print(f"{k} : {v}")
    print("========================================")

    rsw = initialize_rs_devices(arg)
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
