import sys
import numpy as np

from rs_py import get_filepaths

PATH = sys.argv[1]
FPS = int(sys.argv[2])

# PATH = '/code/realsense-simple-wrapper/output/rs_sample_3dev'


if __name__ == "__main__":

    dev_trial_ts_filepaths = get_filepaths(PATH, 'timestamp')

    for dev, trial_ts_filepath in dev_trial_ts_filepaths.items():
        for trial, ts_filepath in trial_ts_filepath.items():

            ts_filepath = ts_filepath[0]
            with open(ts_filepath, 'r') as f:
                lines = f.readlines()

            s_ts = [float(i.split("::")[0]) for i in lines]
            c_ts = [float(i.split("::")[1]) for i in lines]
            d_ts = [float(i.split("::")[2]) for i in lines]

            s_fps = float(len(s_ts))/((s_ts[-1]-s_ts[0])/1e9)
            c_fps = float(len(c_ts))/((c_ts[-1]-c_ts[0])/1e6)
            d_fps = float(len(d_ts))/((d_ts[-1]-d_ts[0])/1e6)

            s_diff = [(i-j)/1e9 for i, j in zip(s_ts[1:], s_ts[:-1])]
            s_diff = [sum(s_diff[x*FPS:(x+1)*FPS]) for x in range(len(s_diff)//FPS)]  # noqa
            hist, bins = np.histogram(s_diff, bins=[x for x in range(0, 11, 1)])  # noqa
            hist_pct = hist / hist.sum() * 100

            print("-"*40)
            print(f"Device    : {dev}")
            print(f"Trial     : {trial}")
            print(f"Color FPS : {c_fps:.2f}")
            print(f"Depth FPS : {d_fps:.2f}")
            print(f"Sys   FPS : {s_fps:.2f}")

            print(f"Sys  Hist : {FPS/bins[0]:6.1f} - {FPS/bins[1]:6.1f} : {bins[0]:6.1f} - {bins[1]:6.1f} => {hist[0]} ({hist_pct[0]:.2f})")  # noqa
            for i in range(1, len(hist)):
                print(f"          : {FPS/bins[i]:6.1f} - {FPS/bins[i+1]:6.1f} : {bins[i]:6.1f} - {bins[i+1]:6.1f} => {hist[i]} ({hist_pct[i]:.2f})")  # noqa
