import sys
import os

import pandas as pd
import numpy as np

# PATH = '/code/realsense-simple-wrapper/output/221010'

PATH = sys.argv[1]

if __name__ == "__main__":

    for trial in sorted(os.listdir(PATH)):
        print("-"*80)
        print(f"Trial           : {trial}")

        device_dir = os.path.join(PATH, trial, 'bbraun_perfusor')
        for csv_file in sorted(os.listdir(device_dir)):
            print(f"CSV file        : {csv_file}")
            df = pd.read_csv(os.path.join(device_dir, csv_file))
            ts = df['Timestamp_ms'].to_numpy()/1000
            ts_fps = float(len(ts))/((ts[-1]-ts[0]))
            ts_diff = ts[1:] - ts[:-1]
            if(ts_diff.size != 0):
                if ts_diff.max() > 10:
                    hist, bins = np.histogram(
                        ts_diff,
                        bins=[x for x in range(0, 11)] + [ts_diff.max()]
                    )
                else:
                    hist, bins = np.histogram(
                        ts_diff,
                        bins=[x for x in range(0, int(ts_diff.max()*2))]
                    )
                print(f"FPS             : {ts_fps:.2f}")
                print(f"bbraun_perfusor : {bins[0]:7.3f} - {bins[1]:7.3f} => {hist[0]}")  # noqa
                for i in range(1, len(hist)):
                    print(f"                : {bins[i]:7.3f} - {bins[i+1]:7.3f} => {hist[i]}")  # noqa

        device_dir = os.path.join(PATH, trial, 'ge_monitor')
        for csv_file in sorted(os.listdir(device_dir)):
            print(f"CSV file        : {csv_file}")
            df = pd.read_csv(os.path.join(device_dir, csv_file))
            if "ECG" in csv_file:
                ts = df.iloc[:, 2].to_numpy()/1000
            elif "PLETH" in csv_file:
                ts = df.iloc[:, 3].to_numpy()/1000
            else:
                df = pd.read_csv(os.path.join(
                    device_dir, csv_file), skiprows=[0], header=None)
                ts = df.iloc[:, 1].to_numpy()/1000
            ts_fps = float(len(ts))/((ts[-1]-ts[0]))
            ts_diff = ts[1:] - ts[:-1]
            if(ts_diff.size != 0):
                if ts_diff.max() > 10:
                    hist, bins = np.histogram(
                        ts_diff,
                        bins=[x for x in range(0, 11)] + [ts_diff.max()]
                    )
                else:
                    max_bin = np.ceil(ts_diff.max()).astype(int)*2
                    hist, bins = np.histogram(
                        ts_diff,
                        bins=[x for x in range(0, max_bin)]
                    )
                print(f"FPS             : {ts_fps:.2f}")
                print(f"ge_monitor      : {bins[0]:7.3f} - {bins[1]:7.3f} => {hist[0]}")  # noqa
                for i in range(1, len(hist)):
                    print(f"                : {bins[i]:7.3f} - {bins[i+1]:7.3f} => {hist[i]}")  # noqa
