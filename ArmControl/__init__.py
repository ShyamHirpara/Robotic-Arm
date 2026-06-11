import requests

# url = "http://192.168.4.1/"
class ArmControl:
    def __init__(self, url = "http://192.168.4.1/"):
        self.base_url = url
        model = "Niryo"
        print("Arm Control")

    def open_gripper(self):
        end_point = "gripper?action=open"

        url = self.base_url + end_point
        response = requests.get(url)
        return response

    def close_gripper(self):
        end_point = "gripper?action=close"

        url = self.base_url + end_point
        response = requests.get(url)
        return response

    # def set_gripper_angle(self, angle):
    #     angle = max(0, min(85, angle))
    #     url = self.base_url + f"gripper?angle={angle}"
    #     return requests.get(url)


    def move_joint(self,num=0,angle=0):
        end_point = "stepper?num="+ str(num) +"&angle=" + str(angle)
        url = self.base_url + end_point
        try:
            response = requests.get(url, timeout=2)
            return response
        except: return None

    def get_current_position(self):
        url = self.base_url + "current_position"
        try:
            response = requests.get(url, timeout=2)
            return response.json()
        except: return None

    def set_stepper_delay(self,num=0 ,delay=200):
        end_point = "set_delay?num="+ str(num) +"&delay=" + str(delay)
        try:
            response = requests.get(url, timeout=2)
            return response
        except: return None

    # requests.post(f"{url}/stepper?num=3&angle={30}", timeout=5)

# print("H")
#
# if __name__ == "__main__":
#
#     class Human:
#         def __init__(self):
#             self.base_url = "http://192.168.4.1/"
#
#     # arm_control = ArmControl()
#     # arm_control.open_gripper()
#     print("G")
