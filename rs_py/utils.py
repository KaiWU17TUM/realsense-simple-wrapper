import argparse
import time


def str2bool(v) -> bool:
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def printout(msg: str, mode: str):
    if mode == 'i':
        print(f"[{time.ctime()}][INFO] : {msg}")
    if mode == 'w':
        print(f"[{time.ctime()}][WARN] : {msg}")
    if mode == 'e':
        print(f"[{time.ctime()}][ERRO] : {msg}")
    if mode == 'x':
        print(f"[{time.ctime()}][EXCE] : {msg}")


if __name__ == "__main__":
    printout("Look Here", mode='i')
    printout("Look Here", mode='w')
    printout("Look Here", mode='e')
