#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np
import os

def gen_golden_data_simple():
    input_a = np.random.uniform(1, 10, [1024, 256]).astype(np.float16)
    input_b = np.random.uniform(-100, 100, [256, 640]).astype(np.float16)
    input_bias = np.random.uniform(-100, 100, [640]).astype(np.float32)
    alpha = 0.001
    golden = (np.matmul(input_a.astype(np.float32), input_b.astype(np.float32)) + input_bias).astype(np.float32)
    golden = np.where(golden >= 0,golden, golden * alpha)
    os.system("mkdir -p input")
    os.system("mkdir -p output")
    input_a.tofile("./input/input_a.bin")
    input_b.tofile("./input/input_b.bin")
    input_bias.tofile("./input/input_bias.bin")
    golden.tofile("./output/golden.bin")

if __name__ == "__main__":
    gen_golden_data_simple()
