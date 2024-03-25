#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np
import os

def gen_golden_data_simple():
    M = 1024
    N = 640
    K = 256
    workspacesize = 96 * 1024 * 1024
    input_a = np.random.uniform(-10, 10, [M, K]).astype(np.float16)
    input_b = np.random.uniform(-100, 100, [K, N]).astype(np.float16)
    workspace = np.zeros([workspacesize]).astype(np.float32)
    input_bias = np.random.uniform(-100, 100, [N]).astype(np.float32)
    alpha = 0.001
    golden = (np.matmul(input_a.astype(np.float32), input_b.astype(np.float32)) + input_bias).astype(np.float32)
    golden = np.where(golden >= 0,golden, golden * alpha)
    os.system("mkdir -p input")
    os.system("mkdir -p output")
    input_a.tofile("./input/x1_gm.bin")
    input_b.tofile("./input/x2_gm.bin")
    workspace.tofile("./input/workspace.bin")
    input_bias.tofile("./input/bias.bin")
    golden.tofile("./output/golden.bin")

if __name__ == "__main__":
    gen_golden_data_simple()
