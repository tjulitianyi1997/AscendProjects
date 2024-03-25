import os
import sys
import numpy as np

loss = 1e-3


def verify_result(real_result, golden,x ,y):
    result = np.fromfile(real_result, dtype=np.float16)
    golden = np.fromfile(golden, dtype=np.float16)
    x = np.fromfile(x, dtype=np.float16)
    y = np.fromfile(y, dtype=np.float16)
    for i in range(len(result)):
        diff = abs(result[i] - golden[i])
        if (diff > loss) and (diff / golden[i] > loss):
            error_message = "x = {} , y = {}, output[{}] is {}, expect {}".format(x[i],y[i],i, result[i], golden[i])
            print(error_message)
            return False
    
    print("test pass")
    return True


if __name__ == '__main__':
    verify_result(sys.argv[1], sys.argv[2],sys.argv[3],sys.argv[4])
