from queue import Queue
import threading
from vision import vision_loop
from sender import sender_loop

def main():
    command_queue = Queue(maxsize=1)
    shared_state = {"current_degree": 0.0, "command_pending": False}

    t1 = threading.Thread(target=vision_loop, args=(command_queue, shared_state), daemon=True)
    t2 = threading.Thread(target=sender_loop, args=(command_queue, shared_state), daemon=True)

    t1.start()
    t2.start()

    print("Vision + Sender threads started")

    t1.join()
    t2.join()

if __name__ == "__main__":
    main()
