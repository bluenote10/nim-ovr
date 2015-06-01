#***********************************************************************************
#
#Filename    :   OVR_CAPI_0_5_0.h
#Content     :   C Interface to Oculus tracking and rendering.
#Copyright   :   Copyright 2014 Oculus VR, LLC All Rights reserved.
#
#Licensed under the Oculus VR Rift SDK License Version 3.2 (the "License");
#you may not use the Oculus VR Rift SDK except in compliance with the License,
#which is provided at the time of installation or download, or which
#otherwise accompanies this software in either electronic or hard copy form.
#
#You may obtain a copy of the License at
#
#http://www.oculusvr.com/licenses/LICENSE-3.2
#
#Unless required by applicable law or agreed to in writing, the Oculus VR SDK
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.
#
#**********************************************************************************

# @file OVR_CAPI_0_5_0.h
# Exposes all general Rift functionality.

#{.deadCodeElim: on.}
when defined(windows): 
  const 
    libname* = "libOVRRT64_0.dll"
elif defined(macosx): 
  const 
    libname* = "libOVRRT64_0.dylib"
else: 
  const 
    libname* = "libOVRRT64_0.so.5"

type 
  ovrBool* = cchar
  uint32_t* = uint32
  uintptr_t* = ptr uint

proc toBool*(ob: ovrBool): bool =
  if ob == char(0): false else: true
  
const 
  ovrFalse* = 0
  ovrTrue* = 1
  
#-----------------------------------------------------------------------------------
# ***** Simple Math Structures
# A 2D vector with integer components.
type
  Vector2i* = object 
    x*: cint
    y*: cint

# A 2D size with integer components.
type 
  Sizei* = object 
    w*: cint
    h*: cint

# A 2D rectangle with a position and size.
# All components are integers.
type 
  Recti* = object 
    Pos*: Vector2i
    Size*: Sizei

# A quaternion rotation.
type 
  Quatf* = object 
    x*: cfloat
    y*: cfloat
    z*: cfloat
    w*: cfloat

# A 2D vector with float components.
type 
  Vector2f* = object 
    x*: cfloat
    y*: cfloat

# A 3D vector with float components.
type 
  Vector3f* = object 
    x*: cfloat
    y*: cfloat
    z*: cfloat

# A 4x4 matrix with float elements.
type 
  Matrix4f* = object 
    M*: array[4, array[4, cfloat]]

# Position and orientation together.
type 
  Posef* = object 
    Orientation*: Quatf
    Position*: Vector3f

# A full pose (rigid body) configuration with first and second derivatives.
type 
  PoseStatef* = object 
    ThePose*: Posef # The body's position and orientation.
    AngularVelocity*: Vector3f # The body's angular velocity in radians per second.
    LinearVelocity*: Vector3f # The body's velocity in meters per second.
    AngularAcceleration*: Vector3f # The body's angular acceleration in radians per second per second.
    LinearAcceleration*: Vector3f # The body's acceleration in meters per second per second.
    Pad*: cfloat # Unused struct padding.
    TimeInSeconds*: cdouble # Absolute time of this state sample.

# Field Of View (FOV) in tangent of the angle units.
# As an example, for a standard 90 degree vertical FOV, we would
# have: { UpTan = tan(90 degrees / 2), DownTan = tan(90 degrees / 2) }.
type 
  FovPort*  = object 
    UpTan* : cfloat # The tangent of the angle between the viewing vector and the top edge of the field of view.
    DownTan* : cfloat # The tangent of the angle between the viewing vector and the bottom edge of the field of view.
    LeftTan* : cfloat # The tangent of the angle between the viewing vector and the left edge of the field of view.
    RightTan* : cfloat # The tangent of the angle between the viewing vector and the right edge of the field of view.

    
#-----------------------------------------------------------------------------------
# ***** HMD Types
# Enumerates all HMD types that we support.
type 
  HmdType* {.size: sizeof(cint).} = enum 
    Hmd_None = 0,
    Hmd_DK1 = 3,
    Hmd_DKHD = 4,
    Hmd_DK2 = 6, 
    Hmd_BlackStar = 7,
    Hmd_CB = 8,
    Hmd_Other = 9, 
    Hmd_EnumSize = 0x7FFFFFFF

# HMD capability bits reported by device.
type
  HmdCaps* {.size: sizeof(cint).} = enum
    # Read-only flags
    HmdCap_Present           = 0x00000001, # (read only) The HMD is plugged in and detected by the system.
    HmdCap_Available         = 0x00000002, # (read only) The HMD and its sensor are available for ownership use.
                                           #             i.e. it is not already owned by another application.
    HmdCap_Captured          = 0x00000004, # (read only) Set to 'true' if we captured ownership of this HMD.
    HmdCap_ExtendDesktop     = 0x00000008, # (read only) Means the display driver works via acting as an addition display monitor.
    HmdCap_DebugDevice       = 0x00000010, # (read only) Means HMD device is a virtual debug device.
    # Modifiable flags (through ovrHmd_SetEnabledCaps).
    HmdCap_DisplayOff        = 0x00000040, # Turns off HMD screen and output (only if 'ExtendDesktop' is off).
    HmdCap_LowPersistence    = 0x00000080, # HMD supports low persistence mode.
    HmdCap_DynamicPrediction = 0x00000200, # Adjust prediction dynamically based on internally measured latency.
    HmdCap_NoVSync           = 0x00001000, # Support rendering without VSync for debugging.
    HmdCap_NoMirrorToWindow  = 0x00002000, # Disables mirroring of HMD output to the window. This may improve 
                                           # rendering performance slightly (only if 'ExtendDesktop' is off).
    HmdCap_EnumSize          = 0x7FFFFFFF

# Tracking capability bits reported by the device.
# Used with ovrHmd_ConfigureTracking.
type 
  TrackingCaps* {.size: sizeof(cint).} = enum 
    TrackingCap_Orientation      = 0x00000010, # Supports orientation tracking (IMU).
    TrackingCap_MagYawCorrection = 0x00000020, # Supports yaw drift correction via a magnetometer or other means.
    TrackingCap_Position         = 0x00000040, # Supports positional tracking.
    TrackingCap_Idle             = 0x00000100, # Overrides the other flags. Indicates that the application
                                               # doesn't care about tracking settings. This is the internal
                                               # default before ovrHmd_ConfigureTracking is called.
    TrackingCap_EnumSize         = 0x7FFFFFFF

# Distortion capability bits reported by device.
# Used with ovrHmd_ConfigureRendering and ovrHmd_CreateDistortionMesh.
type                        
  DistortionCaps* {.size: sizeof(cint).} = enum
    # 0x01 unused - Previously ovrDistortionCap_Chromatic now enabled permanently.
    DistortionCap_TimeWarp           = 0x00000002, # Supports timewarp.
    # 0x04 unused
    DistortionCap_Vignette           = 0x00000008, # Supports vignetting around the edges of the view.
    DistortionCap_NoRestore          = 0x00000010, # Do not save and restore the graphics and compute state when rendering distortion.
    DistortionCap_FlipInput          = 0x00000020, # Flip the vertical texture coordinate of input images.
    DistortionCap_SRGB               = 0x00000040, # Assume input images are in sRGB gamma-corrected color space.
    DistortionCap_Overdrive          = 0x00000080, # Overdrive brightness transitions to reduce artifacts on DK2+ displays
    DistortionCap_HqDistortion       = 0x00000100, # High-quality sampling of distortion buffer for anti-aliasing
    DistortionCap_LinuxDevFullscreen = 0x00000200, # Indicates window is fullscreen on a device when set. The SDK will automatically apply distortion mesh rotation if needed.
    DistortionCap_ComputeShader      = 0x00000400, # Using compute shader (DX11+ only)
   #DistortionCap_NoTimewarpJit      = 0x800         RETIRED - do not reuse this bit without major versioning changes.
    DistortionCap_TimewarpJitDelay   = 0x00001000, # Enables a spin-wait that tries to push time-warp to be as close to V-sync as possible. WARNING - this may backfire and cause framerate loss - use with caution.
    DistortionCap_ProfileNoSpinWaits = 0x00010000, # Use when profiling with timewarp to remove false positives
    DistortionCap_EnumSize = 0x7FFFFFFF

# Specifies which eye is being used for rendering.
# This type explicitly does not include a third "NoStereo" option, as such is
# not required for an HMD-centered API.
type 
  EyeType* {.size: sizeof(cint).} = enum 
    Eye_Left = 0,
    Eye_Right = 1,
    Eye_Count = 2, 
    Eye_EnumSize = 0x7FFFFFFF

# This is a complete descriptor of the HMD.
type 
  HmdDesc*  = object 
    Handle* : pointer                           # Internal handle of this HMD.
    #/ This HMD's type.
    Type* : HmdType                             # Name string describing the product: "Oculus Rift DK1", etc.
    ProductName* : cstring                      # String describing the manufacturer. Usually "Oculus".
    Manufacturer* : cstring                     # HID Vendor ID of the device.
    VendorId* : cshort                          # HID Product ID of the device.
    ProductId* : cshort                         # Sensor (and display) serial number.
    SerialNumber* : array[24, char]             # Sensor firmware major version number.
    FirmwareMajor* : cshort                     # Sensor firmware minor version number.
    FirmwareMinor* : cshort                     # External tracking camera frustum dimensions (if present).
    CameraFrustumHFovInRadians* : cfloat        # Horizontal field-of-view
    CameraFrustumVFovInRadians* : cfloat        # Vertical field-of-view
    CameraFrustumNearZInMeters* : cfloat        # Near clip distance
    CameraFrustumFarZInMeters* : cfloat         # Far clip distance
    HmdCaps* : cuint                            # Capability bits described by ovrHmdCaps.
    TrackingCaps* : cuint                       # Capability bits described by ovrTrackingCaps.
    DistortionCaps* : cuint                     # Capability bits described by ovrDistortionCaps.
    DefaultEyeFov* : array[2, FovPort]          # The recommended optical FOV for the HMD.
    MaxEyeFov* : array[2, FovPort]              # The maximum optical FOV for the HMD.
    EyeRenderOrder* : array[2, EyeType]         # Preferred eye rendering order for best performance.
                                                # Can help reduce latency on sideways-scanned screens.
    Resolution* : Sizei                         # Resolution of the full HMD screen (both eyes) in pixels.
    WindowsPos* : Vector2i                      # Location of the application window on the desktop (or 0,0).
    DisplayDeviceName* : cstring
    DisplayId* : cint

# Simple type ovrHmd is used in ovrHmd_* calls.
type 
  Hmd* = ptr HmdDesc

# Bit flags describing the current status of sensor tracking.
# The values must be the same as in enum StatusBits
type 
  StatusBits* {.size: sizeof(cint).} = enum 
    Status_OrientationTracked = 0x00000001, # Orientation is currently tracked (connected and in use).
    Status_PositionTracked    = 0x00000002, # Position is currently tracked (false if out of range).
    Status_CameraPoseTracked  = 0x00000004, # Camera pose is currently tracked.
    Status_PositionConnected  = 0x00000020, # Position tracking hardware is connected.
    Status_HmdConnected       = 0x00000080, # HMD Display is available and connected.
    Status_EnumSize           = 0x7FFFFFFF

# Specifies a reading we can query from the sensor.
type 
  SensorData* = object 
    Accelerometer* : Vector3f # Acceleration reading in m/s^2.
    Gyro* : Vector3f          # Rotation rate in rad/s.
    Magnetometer* : Vector3f  # Magnetic field in Gauss.
    Temperature* : cfloat     # Temperature of the sensor in degrees Celsius.
    TimeInSeconds* : cfloat   # Time when the reported IMU reading took place, in seconds.

# Tracking state at a given absolute time (describes predicted HMD pose etc).
# Returned by ovrHmd_GetTrackingState.
type 
  TrackingState* = object 
    HeadPose* : PoseStatef
    CameraPose* : Posef 
    LeveledCameraPose* : Posef
    RawSensorData* : SensorData
    StatusFlags* : cuint
    LastCameraFrameCounter* : uint32_t
    Pad* : uint32_t

# Frame timing data reported by ovrHmd_BeginFrameTiming() or ovrHmd_BeginFrame().
type 
  FrameTiming* = object 
    DeltaSeconds* : cfloat
    Pad* : cfloat
    ThisFrameSeconds* : cdouble
    TimewarpPointSeconds* : cdouble
    NextFrameSeconds* : cdouble
    ScanoutMidpointSeconds* : cdouble
    EyeScanoutSeconds* : array[2, cdouble]

# Rendering information for each eye. Computed by either ovrHmd_ConfigureRendering()
# or ovrHmd_GetRenderDesc() based on the specified FOV. Note that the rendering viewport
# is not included here as it can be specified separately and modified per frame through:
#    (a) ovrHmd_GetRenderScaleAndOffset in the case of client rendered distortion,
# or (b) passing different values via ovrTexture in the case of SDK rendered distortion.
type 
  EyeRenderDesc* = object 
    Eye* : EyeType                        # The eye index this instance corresponds to.
    Fov* : FovPort                        # The field of view.
    DistortedViewport* : Recti            # Distortion viewport.
    PixelsPerTanAngleAtCenter* : Vector2f # How many display pixels will fit in tan(angle) = 1.
    HmdToEyeViewOffset* : Vector3f        # Translation to be applied to view matrix for each eye offset.

# Rendering information for positional TimeWarp.
# Contains the data necessary to properly calculate position info for timewarp matrices
# and also interpret depth info provided via the depth buffer to the timewarp shader
type 
  PositionTimewarpDesc* = object 
    HmdToEyeViewOffset* : array[2, Vector3f] # The same offset value pair provided in ovrEyeRenderDesc.
    NearClip* : cfloat                       
    FarClip* : cfloat

#-----------------------------------------------------------------------------------
# ***** Platform-independent Rendering Configuration
# These types are used to hide platform-specific details when passing
# render device, OS, and texture data to the API.
#
# The benefit of having these wrappers versus platform-specific API functions is
# that they allow game glue code to be portable. A typical example is an
# engine that has multiple back ends, say GL and D3D. Portable code that calls
# these back ends may also use LibOVR. To do this, back ends can be modified
# to return portable types such as ovrTexture and ovrRenderAPIConfig.
type 
  RenderAPIType* {.size: sizeof(cint).} = enum 
    RenderAPI_None,
    RenderAPI_OpenGL,
    RenderAPI_Android_GLES,
    RenderAPI_D3D9,      # Deprecated: Not supported for SDK rendering
    RenderAPI_D3D10,     # Deprecated: Not supported for SDK rendering
    RenderAPI_D3D11,
    RenderAPI_Count,
    #RenderAPI_EnumSize = 0x7FFFFFFF

# Platform-independent part of rendering API-configuration data.
# It is a part of ovrRenderAPIConfig, passed to ovrHmd_Configure.
type 
  RenderAPIConfigHeader* = object 
    API* : RenderAPIType    # The graphics API in use.
    BackBufferSize* : Sizei # Previously named RTSize.
    Multisample* : cint     # The number of samples per pixel.

# Contains platform-specific information for rendering.
type 
  RenderAPIConfig* = object 
    Header* : RenderAPIConfigHeader     # Platform-independent rendering information.
    PlatformData* : array[8, uintptr_t] # Platform-specific rendering information.

# Platform-independent part of the eye texture descriptor.
# It is a part of ovrTexture, passed to ovrHmd_EndFrame.
# If RenderViewport is all zeros then the full texture will be used.
type 
  TextureHeader* = object 
    API* : RenderAPIType    # The graphics API in use.
    TextureSize* : Sizei    # The size of the texture.
    RenderViewport* : Recti # Pixel viewport in texture that holds eye image.

# Contains platform-specific information about a texture.
# Specialized for different rendering APIs in:
#     ovrGLTexture, ovrD3D11Texture
type 
  Texture*  = object 
    Header* : TextureHeader             # Platform-independent data about the texture.
    PlatformData* : array[8, uintptr_t] # Specialized in ovrGLTextureData, ovrD3D11TextureData etc.

# 
# -----------------------------------------------------------------------------------
# ***** API Interfaces
when false:
  proc initializeRenderingShimVersion(requestedMinorVersion: cint): ovrBool
    {.cdecl, importc: "ovr_InitializeRenderingShimVersion", dynlib: libovr.}
  proc initializeRenderingShim(): ovrBool
    {.cdecl, importc: "ovr_InitializeRenderingShim", dynlib: libovr.}

# Flags for Initialize()
type 
  InitFlags* {.size: sizeof(cint).} = enum
    Init_Debug          = 0x00000001, 
    Init_ServerOptional = 0x00000002, 
    Init_RequestVersion = 0x00000004, 
    Init_ForceNoDebug   = 0x00000008

# Logging levels
type 
  LogLevel* {.size: sizeof(cint).} = enum 
    LogLevel_Debug = 0,
    LogLevel_Info = 1,
    LogLevel_Error = 2

# Signature for the logging callback.
# Level is one of the ovrLogLevel constants.
type 
  LogCallback* = proc (level: cint; message: cstring) {.cdecl.}

# Parameters for the ovr_Initialize() call.
type 
  InitParams* = object 
    Flags* : uint32_t 
    RequestedMinorVersion* : uint32_t
    LogCallback* : LogCallback
    ConnectionTimeoutMS* : uint32_t

    
proc initialize*(params: ptr InitParams): ovrBool
  {.cdecl, importc: "ovr_Initialize", dynlib: libname.}

proc shutdown*()
  {.cdecl, importc: "ovr_Shutdown", dynlib: libname.}

proc getVersionString*(): cstring
  {.cdecl, importc: "ovr_GetVersionString", dynlib: libname.}

proc hmdDetect*(): cint
  {.cdecl, importc: "ovrHmd_Detect", dynlib: libname.}

proc hmdCreate*(index: cint): Hmd
  {.cdecl, importc: "ovrHmd_Create", dynlib: libname.}

proc hmdCreateDebug*(`type`: HmdType): Hmd
  {.cdecl, importc: "ovrHmd_CreateDebug", dynlib: libname.}



proc destroy*(hmd: Hmd)
  {.cdecl, importc: "ovrHmd_Destroy", dynlib: libname.}
  
proc getLastError*(hmd: Hmd): cstring
  {.cdecl, importc: "ovrHmd_GetLastError", dynlib: libname.}

proc attachToWindow*(hmd: Hmd,
                     window: pointer, 
                     destMirrorRect: ptr Recti, 
                     sourceRenderTargetRect: ptr Recti): ovrBool
  {.cdecl, importc: "ovrHmd_AttachToWindow", dynlib: libname.}

proc getEnabledCaps*(hmd: Hmd): cuint
  {.cdecl, importc: "ovrHmd_GetEnabledCaps", dynlib: libname.}
proc setEnabledCaps*(hmd: Hmd, hmdCaps: cuint)
  {.cdecl, importc: "ovrHmd_SetEnabledCaps", dynlib: libname.}

    
proc configureTracking*(hmd: Hmd, supportedTrackingCaps: cuint, requiredTrackingCaps: cuint): ovrBool
  {.cdecl, importc: "ovrHmd_ConfigureTracking", dynlib: libname.}

proc recenterPose*(hmd: Hmd)
  {.cdecl, importc: "ovrHmd_RecenterPose", dynlib: libname.}

proc getTrackingState*(hmd: Hmd, absTime: cdouble): TrackingState
  {.cdecl, importc: "ovrHmd_GetTrackingState", dynlib: libname.}

proc getFovTextureSize*(hmd: Hmd, eye: EyeType, fov: FovPort, pixelsPerDisplayPixel: cfloat): Sizei
  {.cdecl, importc: "ovrHmd_GetFovTextureSize", dynlib: libname.}

proc configureRendering*(
    hmd: Hmd,
    apiConfig: ptr RenderAPIConfig,
    distortionCaps: cuint,
    eyeFovIn: array[0..1, FovPort],
    eyeRenderDescOut: var array[0..1, EyeRenderDesc]
  ): ovrBool
  {.cdecl, importc: "ovrHmd_ConfigureRendering", dynlib: libname.}


proc beginFrame*(hmd: Hmd, frameIndex: cuint): FrameTiming
  {.cdecl, importc: "ovrHmd_BeginFrame", dynlib: libname.}

proc endFrame*(hmd: Hmd, renderPose: array[0..1, Posef], eyeTexture: array[0..1, Texture])
  {.cdecl, importc: "ovrHmd_EndFrame", dynlib: libname.}

proc getEyePoses*(
    hmd: Hmd,
    frameIndex: cuint,
    hmdToEyeViewOffset: array[0..1, Vector3f],
    outEyePoses: var array[0..1, Posef],
    outHmdTrackingState: ptr TrackingState
  )
  {.cdecl, importc: "ovrHmd_GetEyePoses", dynlib: libname.}




when false:

  # the following functions are not yet wrapped,
  # mainly because they are deprecated anyway.

  Hmd_GetHmdPosePerEye*(Hmd, hmd, EyeType, eye)

  # Client side distortion stuff

  Hmd_GetRenderDesc*(Hmd, hmd, EyeType, eyeType, FovPort, fov)
  
  type 
    DistortionVertex* {.importc: "ovrDistortionVertex", 
                           header: "OVR_CAPI_0_5_0_modified.h".} = object 
      ScreenPosNDC* : Vector2f #/< [-1,+1],[-1,+1] over the entire framebuffer.
      TimeWarpFactor* : cfloat #/< Lerp factor between time-warp matrices. Can be encoded in Pos.z.
      VignetteFactor* : cfloat #/< Vignette fade factor. Can be encoded in Pos.w.
      TanEyeAnglesR* : Vector2f #/< The tangents of the horizontal and vertical eye angles for the red channel.
      TanEyeAnglesG* : Vector2f #/< The tangents of the horizontal and vertical eye angles for the green channel.
      TanEyeAnglesB* : Vector2f #/< The tangents of the horizontal and vertical eye angles for the blue channel.

  type 
    DistortionMesh* {.importc: "ovrDistortionMesh", 
                         header: "OVR_CAPI_0_5_0_modified.h".} = object 
      pVertexData* : ptr DistortionVertex #/< The distortion vertices representing each point in the mesh.
      pIndexData* : ptr cushort #/< Indices for connecting the mesh vertices into polygons.
      VertexCount* : cuint #/< The number of vertices in the mesh.
      IndexCount* : cuint #/< The number of indices in the mesh.

  Hmd_CreateDistortionMesh*(Hmd, hmd, EyeType, eyeType, FovPort, fov, 
                              unsigned, int, distortionCaps, 
                              DistortionMesh * meshData)
  Hmd_CreateDistortionMeshDebug*(Hmd, hmddesc, EyeType, eyeType, 
                                   FovPort, fov, unsigned, int, 
                                   distortionCaps, DistortionMesh * meshData, 
                                   float, debugEyeReliefOverrideInMetres)
  Hmd_DestroyDistortionMesh*(DistortionMesh * meshData)
  Hmd_GetRenderScaleAndOffset*(FovPort, fov, Sizei, textureSize, 
                                 Recti, renderViewport, Vector2f, 
                                 uvScaleOffsetOut[2])
  Hmd_GetFrameTiming*(Hmd, hmd, unsigned, int, frameIndex)
  Hmd_BeginFrameTiming*(Hmd, hmd, unsigned, int, frameIndex)
  Hmd_EndFrameTiming*(Hmd, hmd)
  Hmd_ResetFrameTiming*(Hmd, hmd, unsigned, int, frameIndex)
  Hmd_GetEyeTimewarpMatrices*(Hmd, hmd, EyeType, eye, Posef, 
                                renderPose, Matrix4f, twmOut[2])
  Hmd_GetEyeTimewarpMatricesDebug*(Hmd, hmddesc, EyeType, eye, Posef, 
                                     renderPose, Quatf, playerTorsoMotion, 
                                     Matrix4f, twmOut[2], double, 
                                     debugTimingOffsetInSeconds)

  # The following might be more interesting to wrap, but HSW
  # handling is also removed in 0.6.0.0
  getTimeInSeconds*()
  Hmd_ProcessLatencyTest*(Hmd, hmd, unsigned, char, rgbColorOut[3])
  Hmd_GetLatencyTestResult*(Hmd, hmd)
  Hmd_GetLatencyTest2DrawColor*(Hmd, hmddesc, unsigned, char, 
                                  rgbColorOut[3])
  type 
    HSWDisplayState* {.importc: "ovrHSWDisplayState", 
                          header: "OVR_CAPI_0_5_0_modified.h".} = object 
      Displayed* : ovrBool
      Pad* : array[8 - sizeof((ovrBool)), char]
      StartTime* : cdouble
      DismissibleTime* : cdouble

  Hmd_GetHSWDisplayState*(Hmd, hmd, HSWDisplayState * hasWarningState)
  Hmd_DismissHSWDisplay*(Hmd, hmd)
  
  Hmd_GetBool*(Hmd, hmd, `const`, char * propertyName, ovrBool, defaultVal)
  Hmd_SetBool*(Hmd, hmd, `const`, char * propertyName, ovrBool, value)
  Hmd_GetInt*(Hmd, hmd, `const`, char * propertyName, int, defaultVal)
  Hmd_SetInt*(Hmd, hmd, `const`, char * propertyName, int, value)
  Hmd_GetFloat*(Hmd, hmd, `const`, char * propertyName, float, defaultVal)
  Hmd_SetFloat*(Hmd, hmd, `const`, char * propertyName, float, value)
  Hmd_GetFloatArray*(Hmd, hmd, `const`, char * propertyName, 
                       float * values, unsigned, int, arraySize)
  Hmd_SetFloatArray*(Hmd, hmd, `const`, char * propertyName, 
                       float * values, unsigned, int, arraySize)
  Hmd_GetString*(Hmd, hmd, `const`, char * propertyName, `const`, 
                   char * defaultVal)
  Hmd_SetString*(Hmd, hmddesc, `const`, char * propertyName, `const`, 
                   char * value)

  traceMessage*(int, level, `const`, char * message)
  Hmd_StartPerfLog*(Hmd, hmd, `const`, char * fileName, `const`, 
                      char * userData1)
  Hmd_StopPerfLog*(Hmd, hmd)



#-------------------------------------------------------------------------------------
# Manually added from OVR_CAPI_Util.h
  
type 
  ProjectionModifier* {.size: sizeof(cint).} = enum
    Projection_None = 0x00,
    Projection_RightHanded = 0x01,
    Projection_FarLessThanNear = 0x02,
    Projection_FarClipAtInfinity = 0x04,
    Projection_ClipRangeOpenGL = 0x08,
  

proc Matrix4f_Projection*(fov: FovPort, znear: cfloat, zfar: cfloat, projectionModFlags: ProjectionModifier): Matrix4f
  {.cdecl, importc: "ovrMatrix4f_Projection", dynlib: libname.}

