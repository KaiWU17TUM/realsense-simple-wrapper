import math
from multiprocessing import Manager, Pool, RLock, Manager
import numpy as np
from time import sleep

from rs_py.rs_wrapper import rs
from rs_py import get_rs_parser
from rs_py import RealsenseWrapper


ctx = Manager().list([rs.context()])


def check_arr_arg_length(arg, length: int):
    if length >= 0:
        assert len(arg) == length
    return len(arg)


def parallel_processing(func, num_of_processes: int, *args):

    arrays, non_arrays = [], []
    c = -1

    for arg in args:
        if isinstance(arg, list):
            c = check_arr_arg_length(arg, c)
            arrays.append(arg)
        elif isinstance(arg, np.ndarray):
            c = check_arr_arg_length(arg, c)
            arrays.append(arg)
        else:
            non_arrays.append(arg)

    num_ele_per_process = math.ceil(c / num_of_processes)

    argument_list = []
    for pid in range(num_of_processes):
        arg_tuple = []
        for ele in non_arrays:
            arg_tuple.append(ele)
        for ele in arrays:
            arg_tuple.append(ele[pid*num_ele_per_process:
                                 (pid+1)*num_ele_per_process])
        arg_tuple.append(pid)
        arg_tuple = tuple(arg_tuple)
        argument_list.append(arg_tuple)

    pool = Pool(processes=num_of_processes,
                initargs=(RLock(),))
    jobs = [pool.apply_async(func, args=arg) for arg in argument_list]
    pool.close()
    result_list = [job.get() for job in jobs]
    return result_list


def main(arg, dev_sn, pid):

    sleep(pid)

    arg = arg[0]
    dev_sn = dev_sn[0]
    print("========================================")
    print(">>>>> args <<<<<")
    print("========================================")
    for k, v in vars(arg).items():
        print(f"{k} : {v}")
    print("========================================")

    rsw = RealsenseWrapper(arg, dev_sn, ctx[0])
    rsw.initialize()
    rsw.set_ir_laser_power(arg.rs_laser_power)
    rsw.save_calibration()

    rsw.dummy_capture(30)

    try:
        c = 0
        while True:
            print("[INFO] : step")
            frames = rsw.step(display=arg.rs_display_frame)
            if not len(frames) > 0:
                print("[WARN] : Empty...")
                continue
            c += 1
            if c > arg.rs_fps * 3:
                break

    except Exception as e:
        print("[ERROR]", e)
        print("[INFO] : Stopping RealSense devices...")
        rsw.stop()

    finally:
        print("[INFO] : Stopping RealSense devices...")
        rsw.stop()


if __name__ == "__main__":
    arg = get_rs_parser().parse_args()
    parallel_processing(main, 3,
                        [arg, arg, arg],
                        ['001622070408', '001622070717', '001622071039'])
    print("[INFO] : Finished")
