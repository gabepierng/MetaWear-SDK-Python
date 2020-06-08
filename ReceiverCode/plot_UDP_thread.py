import multiprocessing 
import subprocess
import sys

def worker(file, args):
    subprocess.Popen(['python', file, args])

if __name__ == "__main__": 
    ports = sys.argv[1:end]                          # use given ports                            
    # ports = ['5002', '5004', '5005']            # OR use preset ports 
    for port in ports: 
        p = multiprocessing.Process(target = worker('plot_UDP_single.py', port))
        p.start()