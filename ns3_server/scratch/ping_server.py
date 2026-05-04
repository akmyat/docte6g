import zmq
from common import message_pb2
import time

def test_server():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    
    print("Sending ping...")
    wrapper = message_pb2.Wrapper()
    req = wrapper.channel_state_request
    req.tx_node = 32
    req.rx_node = 0
    req.time = 9084856102 # The time it hung at
    
    socket.send(wrapper.SerializeToString())
    
    if socket.poll(5000): # 5s timeout
        msg = socket.recv()
        print("Received response!")
    else:
        print("Server HUNG!")

if __name__ == "__main__":
    test_server()
