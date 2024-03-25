#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np
import os

def gen_golden_data_simple():
    negative_slope = np.array(0.0, dtype=np.float32)
    input_x = np.random.uniform(-100, 100, [8, 200, 1024]).astype(np.float32)
    golden = np.where(input_x > 0, input_x, input_x * negative_slope).astype(np.float32)
    os.system("mkdir -p input")
    os.system("mkdir -p output")
    input_x.tofile("./input/input_x.bin")
    golden.tofile("./output/golden.bin")

if __name__ == "__main__":
    gen_golden_data_simple()
