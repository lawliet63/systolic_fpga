import numpy as np
import serial
import time
from PIL import Image
import matplotlib.pyplot as plt
import struct
import os
import argparse

# Command definitions - must match Verilog definitions
CMD_NOOP = 0x00
CMD_WRITE_A = 0x01
CMD_WRITE_B = 0x02
CMD_READ_C = 0x03
CMD_COMPUTE = 0x04
CMD_STATUS = 0x05

# Status codes
STATUS_IDLE = 0x00
STATUS_BUSY = 0x01
STATUS_DONE = 0x02
STATUS_ERROR = 0xFF

class FPGAHardwareAccelerator:
    def __init__(self, port, baud_rate=115200, timeout=1):
        """Initialize communication with the FPGA board."""
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        print(f"Connected to {port} at {baud_rate} baud")
        self.array_size = 3  # Default size of systolic array
        
    def close(self):
        """Close the serial connection."""
        self.ser.close()
        print("Connection closed")
        
    def send_command(self, cmd, addr=0, data=None):
        """Send a command to the FPGA with optional address and data."""
        # Send command byte
        self.ser.write(bytes([cmd]))
        
        # If command requires address, send it
        if cmd in [CMD_WRITE_A, CMD_WRITE_B, CMD_READ_C]:
            # Send address as two bytes (high byte first)
            self.ser.write(bytes([(addr >> 8) & 0xFF, addr & 0xFF]))
            
            # If command is write, send data byte
            if cmd in [CMD_WRITE_A, CMD_WRITE_B] and data is not None:
                self.ser.write(bytes([data & 0xFF]))
        
        # For commands that return data immediately, read response
        if cmd in [CMD_NOOP, CMD_STATUS]:
            response = self.ser.read(1)
            if response:
                return response[0]
            else:
                raise TimeoutError("No response received from FPGA")
        
        # For read command, read the data byte
        if cmd == CMD_READ_C:
            response = self.ser.read(1)
            if response:
                return response[0]
            else:
                raise TimeoutError("No response received from FPGA")
        
        # For write commands, read acknowledgment
        if cmd in [CMD_WRITE_A, CMD_WRITE_B]:
            response = self.ser.read(1)
            if response and response[0] == cmd:
                return True
            else:
                raise ValueError(f"Unexpected response: {response}")
        
        return None
    
    def get_status(self):
        """Get the current status of the FPGA."""
        return self.send_command(CMD_STATUS)
    
    def wait_for_completion(self, timeout=30):
        """Wait until the FPGA computation is complete."""
        start_time = time.time()
        while True:
            status = self.get_status()
            
            if status == STATUS_DONE:
                return True
            elif status == STATUS_ERROR:
                raise RuntimeError("FPGA reported an error during computation")
            elif time.time() - start_time > timeout:
                raise TimeoutError(f"Computation timed out after {timeout} seconds")
                
            # Short delay to avoid flooding the serial port
            time.sleep(0.1)
    
    def set_array_size(self, size):
        """Set the size of the systolic array."""
        if size < 1:
            raise ValueError("Array size must be positive")
        self.array_size = size
        print(f"Systolic array size set to {size}x{size}")
    
    def write_matrix_A(self, matrix):
        """
        Write matrix A to the FPGA.
        The matrix should be a numpy array.
        """
        rows, cols = matrix.shape
        
        if rows != self.array_size or cols != self.array_size:
            raise ValueError(f"Matrix must be {self.array_size}x{self.array_size}")
        
        # Write matrix elements row by row
        for i in range(rows):
            for j in range(cols):
                addr = i * cols + j
                data = int(matrix[i, j]) & 0xFF  # Ensure data fits in one byte
                self.send_command(CMD_WRITE_A, addr, data)
                
        print(f"Matrix A ({rows}x{cols}) written to FPGA")
    
    def write_matrix_B(self, matrix):
        """
        Write matrix B to the FPGA.
        The matrix should be a numpy array.
        """
        rows, cols = matrix.shape
        
        if rows != self.array_size or cols != self.array_size:
            raise ValueError(f"Matrix must be {self.array_size}x{self.array_size}")
        
        # Write matrix elements row by row
        for i in range(rows):
            for j in range(cols):
                addr = i * cols + j
                data = int(matrix[i, j]) & 0xFF  # Ensure data fits in one byte
                self.send_command(CMD_WRITE_B, addr, data)
                
        print(f"Matrix B ({rows}x{cols}) written to FPGA")
    
    def read_matrix_C(self):
        """
        Read result matrix C from the FPGA.
        Returns a numpy array.
        """
        rows = cols = self.array_size
        result = np.zeros((rows, cols), dtype=np.uint8)
        
        # Read matrix elements row by row
        for i in range(rows):
            for j in range(cols):
                addr = i * cols + j
                value = self.send_command(CMD_READ_C, addr)
                result[i, j] = value
                
        print(f"Result matrix C ({rows}x{cols}) read from FPGA")
        return result
    
    def compute(self):
        """
        Trigger matrix multiplication computation on the FPGA.
        """
        # Send compute command
        self.send_command(CMD_COMPUTE)
        print("Matrix multiplication computation started on FPGA")
        
        # Wait for computation to finish
        self.wait_for_completion()
        print("Computation completed")
    
    def matrix_multiply(self, matrix_a, matrix_b):
        """
        Perform matrix multiplication using the FPGA.
        Handles the entire process: write matrices, compute, read result.
        """
        # Write input matrices
        self.write_matrix_A(matrix_a)
        self.write_matrix_B(matrix_b)
        
        # Trigger computation
        self.compute()
        
        # Read result
        return self.read_matrix_C()
    
    def reset(self):
        """
        Reset the FPGA hardware accelerator to its initial state.
        """
        self.send_command(CMD_NOOP)
        print("FPGA reset to initial state")


def main():
    """
    Example usage of the FPGA Hardware Accelerator.
    """
    parser = argparse.ArgumentParser(description='FPGA Matrix Multiplication Accelerator')
    parser.add_argument('--port', type=str, required=True, help='Serial port for FPGA communication')
    parser.add_argument('--size', type=int, default=3, help='Size of matrices (NxN)')
    parser.add_argument('--test', action='store_true', help='Run a test multiplication')
    args = parser.parse_args()
    
    # Create accelerator instance
    accelerator = FPGAHardwareAccelerator(args.port)
    accelerator.set_array_size(args.size)
    
    try:
        if args.test:
            # Create test matrices
            matrix_a = np.random.randint(0, 10, size=(args.size, args.size), dtype=np.uint8)
            matrix_b = np.random.randint(0, 10, size=(args.size, args.size), dtype=np.uint8)
            
            print("\nMatrix A:")
            print(matrix_a)
            
            print("\nMatrix B:")
            print(matrix_b)
            
            # Expected result using numpy
            expected = np.matmul(matrix_a, matrix_b)
            print("\nExpected result (numpy):")
            print(expected)
            
            # FPGA computation
            result = accelerator.matrix_multiply(matrix_a, matrix_b)
            print("\nFPGA result:")
            print(result)
            
            # Compare results
            if np.array_equal(expected, result):
                print("\nSuccess! FPGA result matches expected result.")
            else:
                print("\nError: FPGA result does not match expected result!")
                # Calculate difference for debugging
                diff = expected - result
                print("Difference:")
                print(diff)
        else:
            print("No operation specified. Use --test to run a test multiplication.")
    
    finally:
        # Ensure connection is closed
        accelerator.close()


if __name__ == "__main__":
    main()