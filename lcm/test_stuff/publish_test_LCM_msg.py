import lcm
import time
from test_package import TestMessage  # Replace 'your_message' with your actual message file

def publish_test_message():
    # Create an LCM instance
    lc = lcm.LCM()
    
    # Create a message instance
    msg = TestMessage()
    msg.test_value = 42  # Set a test value; adjust as needed

    while True:
        # Publish the message
        lc.publish("TEST_CHANNEL", msg.encode())
        print("Message published")
        time.sleep(1)  # Adjust the sleep time as needed

if __name__ == "__main__":
    publish_test_message()

