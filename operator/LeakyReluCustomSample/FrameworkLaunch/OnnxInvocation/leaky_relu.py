from onnx import TensorProto
from onnx.helper import (make_model, make_node, make_graph, make_tensor, make_tensor_value_info)
from onnx.checker import check_model

X = make_tensor_value_info("X", TensorProto.FLOAT, [None, None, None])
Y = make_tensor_value_info("Y", TensorProto.FLOAT, [None, None, None])
nodel = make_node("LeakyRelu", ["X"], ["Y"], alpha=0.1)

graph = make_graph([nodel], 'leakyrelu', [X], [Y])

onnx_model = make_model(graph)
check_model(onnx_model)

del onnx_model.opset_import[:]
opset = onnx_model.opset_import.add()
opset.domain = ""
opset.version = 11

with open('leaky_relu.onnx', "wb") as f:
    f.write(onnx_model.SerializeToString())
