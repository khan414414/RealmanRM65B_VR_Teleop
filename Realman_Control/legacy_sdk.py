"""Small, typed binding to RealMan API 4.2.8 for RM65-BI 1.4.x firmware.

The ABI follows the manufacturer's Python example linked in vendor/README.md.
Keep numeric C return codes: the original Python wrapper turns them into strings.
Importing this module does not load a library or connect to a robot.
"""

from __future__ import annotations

import ctypes as ct
import hashlib
from pathlib import Path
import sys
import threading


SDK_VERSION = "4.2.8"
DLL_SHA256 = "63d09b392514b80bd061e9be86c5259de4a263d0600e35b06f7fa6c4e1ee5c03"
DLL_PATH = Path(__file__).parent / "vendor" / "api_4_2_8" / "RM_Base.dll"
Joints = ct.c_float * 6


class Position(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ("x", "y", "z")]


class Quaternion(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ("w", "x", "y", "z")]


class Euler(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ("rx", "ry", "rz")]


class Pose(ct.Structure):
    _fields_ = [("position", Position), ("quaternion", Quaternion), ("euler", Euler)]

    @classmethod
    def from_euler(cls, values):
        return cls(position=Position(*values[:3]), euler=Euler(*values[3:]))

    def to_euler(self):
        return [self.position.x, self.position.y, self.position.z,
                self.euler.rx, self.euler.ry, self.euler.rz]


class Frame(ct.Structure):
    _fields_ = [("name", ct.c_char * 12), ("pose", Pose),
                ("payload", ct.c_float), ("x", ct.c_float),
                ("y", ct.c_float), ("z", ct.c_float)]


class JointStatus(ct.Structure):
    # The callback ABI reserves seven joints even on a six-axis RM65.
    _fields_ = [("joint_current", ct.c_float * 7),
                ("joint_en_flag", ct.c_ubyte * 7),
                ("joint_err_code", ct.c_uint16 * 7),
                ("joint_position", ct.c_float * 7),
                ("joint_temperature", ct.c_float * 7),
                ("joint_voltage", ct.c_float * 7)]


class ForceData(ct.Structure):
    _fields_ = [("force", ct.c_float * 6), ("zero_force", ct.c_float * 6),
                ("coordinate", ct.c_int)]


class RobotStatus(ct.Structure):
    _fields_ = [("errCode", ct.c_int), ("arm_ip", ct.c_char_p),
                ("arm_err", ct.c_uint16), ("joint_status", JointStatus),
                ("force_sensor", ForceData), ("sys_err", ct.c_uint16),
                ("waypoint", Pose)]


RealtimeCallback = ct.CFUNCTYPE(None, RobotStatus)


class LegacyArm:
    """RM65 API 4.2.8 connection; process-wide SDK lifetime is reference counted."""

    _library = None
    _users = 0
    _callback_keepalive = None
    _lock = threading.RLock()

    @classmethod
    def _load_library(cls):
        if sys.platform != "win32" or ct.sizeof(ct.c_void_p) != 8:
            raise RuntimeError("The bundled API 4.2.8 requires Windows 64-bit Python.")
        if hashlib.sha256(DLL_PATH.read_bytes()).hexdigest() != DLL_SHA256:
            raise RuntimeError("The bundled RealMan API 4.2.8 DLL checksum is invalid.")
        lib = ct.CDLL(str(DLL_PATH.resolve()))
        pointer = ct.POINTER
        signatures = {
            "API_Version": ([], ct.c_char_p),
            "RM_API_Init": ([ct.c_int, ct.c_void_p], ct.c_int),
            "RM_API_UnInit": ([], ct.c_int),
            "Arm_Socket_Start": ([ct.c_char_p, ct.c_int, ct.c_int], ct.c_int),
            "Arm_Socket_State": ([ct.c_int], ct.c_int),
            "Arm_Socket_Close": ([ct.c_int], None),
            "Get_Arm_Software_Version": ([ct.c_int] + [ct.c_char_p] * 5, ct.c_int),
            "Get_Current_Arm_State": ([ct.c_int, Joints, pointer(Pose),
                                        pointer(ct.c_uint16), pointer(ct.c_uint16)], ct.c_int),
            "Get_Current_Tool_Frame": ([ct.c_int, pointer(Frame)], ct.c_int),
            "Manual_Set_Tool_Frame": ([ct.c_int, ct.c_char_p, Pose] +
                                        [ct.c_float] * 4 + [ct.c_bool], ct.c_int),
            "Change_Tool_Frame": ([ct.c_int, ct.c_char_p, ct.c_bool], ct.c_int),
            "Movej_Cmd": ([ct.c_int, Joints, ct.c_byte, ct.c_float,
                            ct.c_int, ct.c_bool], ct.c_int),
            "Movel_Cmd": ([ct.c_int, Pose, ct.c_byte, ct.c_float,
                            ct.c_int, ct.c_bool], ct.c_int),
            "Movej_CANFD": ([ct.c_int, Joints, ct.c_bool, ct.c_int], ct.c_int),
            "Movep_CANFD": ([ct.c_int, Pose, ct.c_bool], ct.c_int),
            "Move_Slow_Stop_Cmd": ([ct.c_int, ct.c_bool], ct.c_int),
            "Get_Realtime_Push": ([ct.c_int, pointer(ct.c_int), pointer(ct.c_int),
                                     pointer(ct.c_bool), pointer(ct.c_int), ct.c_char_p], ct.c_int),
            "Set_Realtime_Push": ([ct.c_int, ct.c_int, ct.c_int, ct.c_bool,
                                     ct.c_int, ct.c_char_p], ct.c_int),
            "Realtime_Arm_Joint_State": ([RealtimeCallback], None),
        }
        for name in ("Get_Joint_Degree", "Get_Joint_Min_Pos", "Get_Joint_Max_Pos"):
            signatures[name] = ([ct.c_int, Joints], ct.c_int)
        for name, (args, result) in signatures.items():
            function = getattr(lib, name)
            function.argtypes, function.restype = args, result
        version = lib.API_Version().decode("ascii")
        if version != SDK_VERSION:
            raise RuntimeError(f"Expected RealMan API {SDK_VERSION}, received {version}")
        return lib

    def __init__(self, ip: str, port: int = 8080, timeout_ms: int = 1000):
        address = ip.encode("ascii")
        self._closed = True
        self._callback = None
        with self._lock:
            cls = type(self)
            if cls._library is None:
                cls._library = self._load_library()
            self.library = cls._library
            if cls._users == 0:
                code = self.library.RM_API_Init(65, None)
                if code != 0:
                    raise RuntimeError(f"RM_API_Init failed with code {code}")
            cls._users += 1
            try:
                self.socket = self.library.Arm_Socket_Start(address, port, timeout_ms)
                state = self.library.Arm_Socket_State(self.socket)
                if self.socket <= 0 or state != 0:
                    raise ConnectionError(
                        f"API 4.2.8 failed to connect to RM65-BI at {ip}:{port}; "
                        f"socket={self.socket}, state={state}"
                    )
            except BaseException:
                cls._users -= 1
                if cls._users == 0:
                    self.library.RM_API_UnInit()
                raise
            self._closed = False
            try:
                code, info = self.Get_Arm_Software_Version()
                if code != 0 or not info["Product_version"].startswith("RM65-B"):
                    raise ConnectionError(
                        f"RM65-BI version query failed: SDK code {code}, info={info}"
                    )
                self.software_info = info
            except BaseException:
                self.close()
                raise

    def _call(self, name, *args):
        with self._lock:
            if self._closed:
                raise RuntimeError("RM65-BI connection is closed")
            return getattr(self.library, name)(self.socket, *args)

    def Get_Arm_Software_Version(self):
        buffers = [ct.create_string_buffer(256) for _ in range(5)]
        code = self._call("Get_Arm_Software_Version", *buffers)
        keys = ("Plan_version", "Ctrl_version", "Real-time_Kernal1",
                "Real-time_Kernal2", "Product_version")
        return code, dict(zip(keys, (b.value.decode("ascii") for b in buffers)))

    def _joints(self, name):
        joints = Joints()
        code = self._call(name, joints)
        return code, list(joints)

    def Get_Joint_Degree(self):
        return self._joints("Get_Joint_Degree")

    def Get_Joint_Min_Pos(self):
        return self._joints("Get_Joint_Min_Pos")

    def Get_Joint_Max_Pos(self):
        return self._joints("Get_Joint_Max_Pos")

    def Get_Current_Arm_State(self):
        joints, pose = Joints(), Pose()
        arm_error, system_error = ct.c_uint16(), ct.c_uint16()
        code = self._call("Get_Current_Arm_State", joints, ct.byref(pose),
                          ct.byref(arm_error), ct.byref(system_error))
        return code, list(joints), pose.to_euler(), arm_error.value, system_error.value

    def Get_Current_Tool_Frame(self):
        frame = Frame()
        code = self._call("Get_Current_Tool_Frame", ct.byref(frame))
        return code, frame

    def Manual_Set_Tool_Frame(self, name, pose, payload, x, y, z):
        return self._call("Manual_Set_Tool_Frame", name.encode("utf-8"),
                          Pose.from_euler(pose), payload, x, y, z, True)

    def Change_Tool_Frame(self, name):
        return self._call("Change_Tool_Frame", name.encode("utf-8"), True)

    def Movej_Cmd(self, joints, speed, radius, connect, block):
        return self._call("Movej_Cmd", Joints(*joints), speed, radius, connect, block)

    def Movel_Cmd(self, pose, speed, radius, connect, block):
        return self._call("Movel_Cmd", Pose.from_euler(pose), speed, radius, connect, block)

    def Movej_CANFD(self, joints, follow):
        return self._call("Movej_CANFD", Joints(*joints), follow, 0)

    def Movep_CANFD(self, pose, follow):
        return self._call("Movep_CANFD", Pose.from_euler(pose), follow)

    def Move_Slow_Stop_Cmd(self):
        return self._call("Move_Slow_Stop_Cmd", True)

    def Get_Realtime_Push(self):
        cycle, port = ct.c_int(), ct.c_int()
        enable, force = ct.c_bool(), ct.c_int(-1)
        ip = ct.create_string_buffer(256)
        code = self._call("Get_Realtime_Push", ct.byref(cycle), ct.byref(port),
                          ct.byref(enable), ct.byref(force), ip)
        return code, {"cycle": cycle.value, "port": port.value, "enable": enable.value,
                      "force_coordinate": force.value, "ip": ip.value.decode("ascii")}

    def Set_Realtime_Push(self, config):
        return self._call("Set_Realtime_Push", config["cycle"], config["port"],
                          config["enable"], config["force_coordinate"],
                          config["ip"].encode("ascii"))

    def Realtime_Arm_Joint_State(self, callback):
        with self._lock:
            if self._closed:
                raise RuntimeError("RM65-BI connection is closed")
            self._callback = callback  # Keep the function pointer alive for C.
            # The C listener is global and may outlive the owning connection.
            type(self)._callback_keepalive = callback
            self.library.Realtime_Arm_Joint_State(callback)

    def close(self):
        with self._lock:
            if self._closed:
                return 0
            self._closed = True
            try:
                self.library.Arm_Socket_Close(self.socket)
            finally:
                cls = type(self)
                cls._users -= 1
                if cls._users == 0:
                    self.library.RM_API_UnInit()
                    cls._callback_keepalive = None
            return 0
