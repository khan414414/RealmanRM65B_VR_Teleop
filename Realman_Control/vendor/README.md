# RealMan legacy controller SDK

`api_4_2_8/RM_Base.dll` is the manufacturer's **API 4.2.8, Windows x86-64**
library. It is used by this branch to communicate with RM65-BI firmware 1.4.x.
This vendor binary remains subject to its manufacturer's rights; the project's
MIT license does not grant additional rights to the binary.

Source: the attachment to the [official RealMan forum example](https://bbs.realman-robotics.cn/question/316.html),
which specifies SDK version 4.2.8:

https://bbs.realman-robotics.cn/uploads/20240523/f04ca5db729b83bd6dfee04e043fee1a.zip

The DLL was extracted from `ECO65夹取试管demo/RM_Base.dll`. Its exported
`API_Version()` returns `4.2.8`; its PE machine type is `0x8664` (AMD64).
The Python function signatures and callback structures in `legacy_sdk.py`
follow `robotic_arm.py` from the same attachment. The example program itself is
not imported or run.

DLL size: 568832 bytes.

SHA-256 (checked before loading):

```text
63d09b392514b80bd061e9be86c5259de4a263d0600e35b06f7fa6c4e1ee5c03
```

The bundled library supports 64-bit Windows Python. Other platforms require a
separate, matching manufacturer library and ABI verification.

API2 (`Robotic_Arm==1.1.6`) is retained exclusively for the offline `Algo` class:
it supplies official IK/FK and predictive singularity/self-collision checks.
Controller communication continues to use API 4.2.8.
No `RoboticArm`, `rm_init`, or `rm_create_robot_arm` call is used on this branch.
