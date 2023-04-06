import argparse
import time


def str2bool(v) -> bool:
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
    

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKCYAN = '\033[96m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'


def printout(msg: str, mode: str):
    if mode == 'i':
        print(f"{OKGREEN}[{time.ctime()} INFO] : {msg}{ENDC}")
    if mode == 'w':
        print(f"{WARNING}[{time.ctime()} WARN] : {msg}{ENDC}")
    if mode == 'e':
        print(f"{FAIL   }[{time.ctime()} ERRO] : {msg}{ENDC}")
    if mode == 'x':
        print(f"{WARNING}[{time.ctime()} EXCE] : {msg}{ENDC}")


if __name__ == "__main__":
    printout("Look Here", mode='i')
    printout("Look Here", mode='w')
    printout("Look Here", mode='e')
