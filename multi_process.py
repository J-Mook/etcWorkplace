# import os
# from multiprocessing import Process

# def doubler(_num):
#     rslt = _num * 2
#     pid = os.getpid()
#     print("{0} doubled to {1} by process id: {2}".format(_num, rslt, pid))

# if __name__ == "__main__":
#     number = [x*2 for x in range(100000)]
#     procs = list()

#     for i, n in enumerate(number):
#         proc = Process(target=doubler(n))
#         # proc = doubler(n)
#         procs.append(proc)
#         proc.start()

#     for proc in procs:
#         proc.join()


import multiprocessing
import os

print(os.cpu_count())
print(multiprocessing.cpu_count())
