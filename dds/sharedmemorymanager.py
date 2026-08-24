import json
import time
import threading
from typing import Dict, Any, Optional
from multiprocessing import shared_memory


class SharedMemoryManager:
    """Shared memory manager"""
    
    def __init__(self, name: str = None, size: int = 512):
        """Initialize shared memory manager
        
        Args:
            name: shared memory name, if None, create new one
            size: shared memory size (bytes)
        """
        self.size = size
        self.lock = threading.RLock()  # reentrant lock
        
        if name:
            try:
                self.shm = shared_memory.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
            except FileNotFoundError:
                self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
                self.shm_name = name
                self.created = True
        else:
            self.shm = shared_memory.SharedMemory(create=True, size=size)
            self.shm_name = self.shm.name
            self.created = True
    
    def write_data(self, data: Dict[str, Any]) -> bool:
        """Write data to shared memory
        
        Args:
            data: data to write
            
        Returns:
            bool: write success or not
        """
        try:
            with self.lock:
                json_str = json.dumps(data)
                json_bytes = json_str.encode('utf-8')
                
                if len(json_bytes) > self.size - 8:  # reserve 8 bytes for length and timestamp
                    print(f"Warning: Data too large for shared memory ({len(json_bytes)} > {self.size - 8})")
                    return False
                
                # write timestamp (4 bytes) and data length (4 bytes)
                timestamp = int(time.time()) & 0xFFFFFFFF  # 32-bit timestamp, use bitmask to ensure in range
                self.shm.buf[0:4] = timestamp.to_bytes(4, 'little')
                self.shm.buf[4:8] = len(json_bytes).to_bytes(4, 'little')
                
                # write data
                self.shm.buf[8:8+len(json_bytes)] = json_bytes
                return True
                
        except Exception as e:
            print(f"Error writing to shared memory: {e}")
            return False
    
    def read_data(self) -> Optional[Dict[str, Any]]:
        """Read data from shared memory
        
        Returns:
            Dict[str, Any]: read data dictionary, return None if failed
        """
        try:
            with self.lock:
                # read timestamp and data length
                timestamp = int.from_bytes(self.shm.buf[0:4], 'little')
                data_len = int.from_bytes(self.shm.buf[4:8], 'little')
                
                if data_len == 0:
                    return None
                
                # read data
                json_bytes = bytes(self.shm.buf[8:8+data_len])
                data = json.loads(json_bytes.decode('utf-8'))
                data['_timestamp'] = timestamp  # add timestamp information
                return data
                
        except Exception as e:
            print(f"Error reading from shared memory: {e}")
            return None
    
    def get_name(self) -> str:
        """Get shared memory name"""
        return self.shm_name
    
    def cleanup(self):
        """Clean up shared memory"""
        if hasattr(self, 'shm') and self.shm:
            self.shm.close()
            if self.created:
                try:
                    self.shm.unlink()
                except:
                    pass
    
    def __del__(self):
        """Destructor"""
        self.cleanup()
