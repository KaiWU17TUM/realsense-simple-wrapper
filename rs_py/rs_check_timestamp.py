import os

PATH = '/tmp/testing15'


def get_timestamp(base_path):
    path = {}
    for device in sorted(os.listdir(base_path)):
        device_path = os.path.join(base_path, device)
        for ts in sorted(os.listdir(device_path)):
            calib_path = os.path.join(base_path, device, ts, 'timestamp')
            for filename in sorted(os.listdir(calib_path)):
                path[device] = os.path.join(calib_path, filename)
    return path


if __name__ == "__main__":

    dev_ts_file = get_timestamp(PATH)

    for dev, ts_file in dev_ts_file.items():
        with open(ts_file, 'r') as f:
            lines = f.readlines()

        c_ts = [float(i.split("::")[0]) for i in lines]
        d_ts = [float(i.split("::")[1]) for i in lines]

        print(f"Device    : {dev}")
        print(f"Color FPS : {float(len(c_ts))/((c_ts[-1]-c_ts[0])/1e6):.2f}")
        print(f"Depth FPS : {float(len(d_ts))/((d_ts[-1]-d_ts[0])/1e6):.2f}")
