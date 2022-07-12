import sys

from rs_py import get_filepaths

PATH = sys.argv[1]

# PATH = '/code/realsense-simple-wrapper/output/rs_sample_3dev'


if __name__ == "__main__":

    dev_trial_ts_filepaths = get_filepaths(PATH, 'timestamp')

    for dev, trial_ts_filepath in dev_trial_ts_filepaths.items():
        for trial, ts_filepath in trial_ts_filepath.items():

            ts_filepath = ts_filepath[0]
            with open(ts_filepath, 'r') as f:
                lines = f.readlines()

            c_ts = [float(i.split("::")[1]) for i in lines]
            d_ts = [float(i.split("::")[2]) for i in lines]

            c_fps = float(len(c_ts))/((c_ts[-1]-c_ts[0])/1e6)
            d_fps = float(len(d_ts))/((d_ts[-1]-d_ts[0])/1e6)

            print("-"*40)
            print(f"Trial     : {trial}")
            print(f"Device    : {dev}")
            print(f"Color FPS : {c_fps:.2f}")
            print(f"Depth FPS : {d_fps:.2f}")
