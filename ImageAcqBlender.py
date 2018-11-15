
#================================================================#
import bpy
import bpy_extras
import numpy
import numpy as np
import math
from math import *
import mathutils
import os
from math import radians
from math import pi
from math import sin
from math import cos
from mathutils import Matrix
from mathutils import Vector
#================================================================#
def AddCamTopDown(i):
    bpy.ops.object.camera_add(location=[0,0,cos(0)])
    tmp = "Camera"+"%.3d" %i
    bpy.data.objects["Camera"].name = tmp
    bpy.data.cameras["Camera"].name = tmp
    bpy.data.objects[tmp].scale[0] = 0.020
    update_camera(bpy.data.objects[tmp])
    make_iphone6_of_the_camera(bpy.data.cameras[tmp])
    i+=1
    bpy.ops.object.camera_add(location=[0,0,cos(pi)])
    tmp = "Camera"+"%.3d" %i
    bpy.data.objects["Camera"].name = tmp
    bpy.data.cameras["Camera"].name = tmp
    bpy.data.objects[tmp].scale[0] = 0.020
    update_camera(bpy.data.objects[tmp])
    make_iphone6_of_the_camera(bpy.data.cameras[tmp])
    num_cam = i
    return num_cam

def import_residual_limb(ply_name):
    file_loc="C:/Users/.../plyfiles/%s.ply" % ply_name
    bpy.ops.import_mesh.ply(filepath = file_loc, filter_glob="*.ply", directory="C:/Users/.../plyfiles")
    #bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')

#This function points all the cameras towards the scene center
def update_camera(camera, focus_point=mathutils.Vector((0.0, 0.0, 0.0))):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.

    source: https://blender.stackexchange.com/questions/100414/how-to-set-camera-location-in-the-scene-while-pointing-towards-an-object-with-a
    """
    looking_direction = camera.location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.rotation_euler = rot_quat.to_euler()
    camera.location = rot_quat * mathutils.Vector((0.0, 0.0, r)) #r is the distance from the center of the scene to the center of the camera

# Calculates Rotation Matrix given euler angles.
def Eul2Rotm(theta) :

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),     0],
                    [math.sin(theta[2]),     math.cos(theta[2]),     0],
                    [0,                      0,                      1]
                    ])

    R = np.dot(R_x, np.dot( R_y, R_z ))
    return R

#This function give the camera the iPhone 6 values
def make_iphone6_of_the_camera(camera):
    """These parameters are found online: https://www.devicespecifications.com/en/model/5d342ce2"""
    camera.sensor_width = 4.80 #in mm
    camera.sensor_height = 3.60 #in mm
    camera.lens = 4.15 #in mm
    camera.sensor_fit = 'HORIZONTAL'
    bpy.data.scenes["Scene"].render.resolution_percentage = 100
    bpy.data.scenes["Scene"].render.resolution_x = 3264 # number of pixels in the x direction on the sensor
    bpy.data.scenes["Scene"].render.resolution_y = 2448 # number of pixels in the y direction on the sensor

# This function calculates the K matrix
def get_calibration_matrix_K_from_blender(cam):
    """source: https://blender.stackexchange.com/questions/38009/3x4-camera-matrix-from-blender-camera"""
    scene = bpy.data.scenes["Scene"]
    f_in_mm = cam.lens #focal length in mm
    resolution_x_in_px = scene.render.resolution_x # number of pixels in the x direction. 3264 for iPhone 6
    resolution_y_in_px = scene.render.resolution_y # number of pixels in the y direction. 2448 for iPhone 6
    sensor_width_in_mm = cam.sensor_width #sensor width in mm
    sensor_height_in_mm = cam.sensor_height #sensor height in mm

    # the sensor width (resolution_x_in_px) is fixed (sensor fit is horizontal),
    # Assuming fit image- and FOV-angle to the width of the sensor (sensor_fit = 'HORIZONTAL')
    # Assuming resolution_percentage is 100% ~ scale is 1
    # Assuming pixel_aspect_ratio = 1
    s_u = resolution_x_in_px / sensor_width_in_mm # = 680 px per mm for iPhone 6
    s_v = resolution_y_in_px / sensor_height_in_mm # = 680 px per mm for iPhone 6


    # Parameters of intrinsic calibration matrix K
    """The parameters alpha_u = f * s_u and alpha_v = f * s_v represent focal length in terms of pixels,
    where s_u and s_v are the scale factors relating pixels to distance and f is the focal length in terms of distance.
    u_0 and v_0 represent the principal point, which would be ideally in the centre of the image.
    To overcome the rendererror, the values are rounded to 2 decimals places.
    source: https://en.wikipedia.org/wiki/Camera_resectioning"""

    alpha_u = round((f_in_mm * s_u),2)
    alpha_v = round((f_in_mm * s_v),2)
    u_0 = round((resolution_x_in_px / 2),2) #the "/2" is to find the x coordinate in pixels of the centre of the image
    v_0 = round((resolution_y_in_px / 2),2) #the "/2" is to find the y coordinate in pixels of the centre of the image
    skew = 0 # only use rectangular pixels

    K = np.array([[alpha_u, skew   ,  u_0],
                  [   0   , alpha_v,  v_0],
                  [   0   , 0      ,    1]])

    return K

# Returns camera rotation and translation matrices from Blender.
def get_3x4_RT_matrix_from_blender(cam):
    """The important thing to remember about the extrinsic matrix is that it describes how
    the world is transformed relative to the camera. This is often counter-intuitive, because
    we usually want to specify how the camera is transformed relative to the world.

    It's often more natural to specify the camera's pose directly rather than specifying
    how world points should transform to camera coordinates. Luckily, building an extrinsic
    camera matrix this way is easy: just build a rigid transformation matrix that describes
    the camera's pose and then take it's inverse. If you have an orthonormal matrix (square matrix)
    (e.g: a rotation matrix) then you use the transpose to get the inverse transformation
    (e.g: the inverse rotation)."""
    # cam = bpy.data.objects["Camera001"] (for exmple)
    # C, R_c = bpy.data.objects["Camera001"].matrix_world.decompose()[0:2]

    # Decompose the camera parameters in location C and rotation R_c (in Quaternion coor, default)
    C, R_c = cam.matrix_world.decompose()[0:2]
    # Matlab has a default rotation order of ZYX. The Quaternion coor are therefore transformed to ZYX euler angles.
    R_c = R_c.to_euler("ZYX")
    # The rotation matrix is constructed with a previously defined function Eul2Rotm
    R_c =  Eul2Rotm(R_c).round(decimals=6)
    # Take the transpose as described in above. R_cam2world = R_c^T
    R_cam2world = R_c.round(decimals=6).transpose()
    # Convert camera location to translation vector. t = -R_c^T*C = -R_cam2world*C
    t_cam2world = np.negative(R_cam2world.dot(C))
    # Make z positive in the lookat direction and make y positive in the down direction. x remains unhanged. Right hand rule.
    rot2cv = np.array([[ 1,  0,  0],
                       [ 0, -1,  0],
                       [ 0,  0, -1]])

    R_cam2world = rot2cv.dot(R_cam2world)
    t_cam2world = rot2cv.dot(t_cam2world)

    '''      _______________
            |        x      | Right hand rule (thumb in x direction, middle finger is the lookat direction)
            |       --->    |
            |     y|        |
            |      v        |
             ---------------
    '''

    # put into 3x4 matrix
    RT = np.array([[R_cam2world[0][0], R_cam2world[0][1], R_cam2world[0][2], t_cam2world[0]],
                   [R_cam2world[1][0], R_cam2world[1][1], R_cam2world[1][2], t_cam2world[1]],
                   [R_cam2world[2][0], R_cam2world[2][1], R_cam2world[2][2], t_cam2world[2]]])
    return RT

def get_3x4_P_matrix_from_blender(cam):
    K = get_calibration_matrix_K_from_blender(cam.data) #cam = bpy.data.objects["Camera001"] --> cam.data = bpy.data.objects["Camera001"].data = bpy.data.cameras["Camera001"]
    RT = get_3x4_RT_matrix_from_blender(cam)
    return K.dot(RT), K, RT

def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step


#================================================================#

os.system('cls')

txtLoc = "C:/Users/.../BlenderOutputParams/"

test = os.listdir(txtLoc)
for item in test:
    if item.endswith(".png"):
        os.remove(os.path.join(txtLoc, item))

open(txtLoc + "Pmat.txt", 'w').close()
open(txtLoc + "RTmat.txt", 'w').close()
open(txtLoc + "Kmat.txt", 'w').close()

item='CAMERA'
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type=item)
bpy.ops.object.delete()

for block in bpy.data.cameras:
    if block.users == 0:
        bpy.data.cameras.remove(block)

item='MESH'
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type=item)
bpy.ops.object.delete()

ply_name = "......" #from plyfiles. (no .ply in the name)
import_residual_limb(ply_name)

r=.5 # metric depends on the scene data [in this case [m]]
bpy.data.objects[ply_name].location[0]=0.00
bpy.data.objects[ply_name].location[1]=0
bpy.data.objects[ply_name].location[2]=0
bpy.data.objects[ply_name].rotation_euler[0]=0

'''
# camera setup defined by sperical coordinates
step = pi/10
i = 1
for theta in frange((0.5*pi),(1.51*pi),step):
    for phi in frange(step,(pi),step+0.0001):
        x = r * sin(phi)*cos(theta)
        y = r * sin(phi)*sin(theta)
        z = r * cos(phi)
        bpy.ops.object.camera_add(location=[x,y,z])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])
        i+=1
print("i = ", i)
num_cam = AddCamTopDown(i)
print("num_cam = ", num_cam)
'''

num_cam = 3
step_theta = (2*pi)/(num_cam+1)
theta = .5*pi
phi = .5*pi
for i in range(1, (num_cam + 1)):
    x = r * sin(phi)*cos(theta)
    y = r * sin(phi)*sin(theta)
    z = r * cos(phi)
    if i != 3:
        bpy.ops.object.camera_add(location=[x,y,z])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])
        theta = theta + step_theta
    else:
        bpy.ops.object.camera_add(location=[0,0,r])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])

'''
# three camera setup in orthognal directions
num_cam = 3
step_theta = (2*pi)/(num_cam+1)
theta = .5*pi
phi = .5*pi
for i in range(1, (num_cam + 1)):
    if i == 1:
        r = 0.55
        x = r * sin(phi)*cos(theta)
        y = r * sin(phi)*sin(theta)
        z = r * cos(phi)
        bpy.ops.object.camera_add(location=[x,y,z])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])
        theta = theta + step_theta
    elif i == 2:
        r = 0.50
        x = r * sin(phi)*cos(theta)
        y = r * sin(phi)*sin(theta)
        z = r * cos(phi)
        bpy.ops.object.camera_add(location=[x,y,z])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])
        theta = theta + step_theta
    else:
        r = .45
        bpy.ops.object.camera_add(location=[0,0,r])
        tmp = "Camera"+"%.3d" %i
        bpy.data.objects["Camera"].name = tmp
        bpy.data.cameras["Camera"].name = tmp
        bpy.data.objects[tmp].scale[0] = 0.020
        update_camera(bpy.data.objects[tmp])
        make_iphone6_of_the_camera(bpy.data.cameras[tmp])
'''


np.set_printoptions(suppress=True)

Pfile = open(txtLoc + "Pmat.txt", 'a')
RTfile = open(txtLoc + "RTmat.txt", 'a')
for i in range(1,(num_cam + 1)):
    context = bpy.context
    scene = context.scene
    tmp = "Camera"+"%.3d" %i
    cam = bpy.data.objects[tmp]
    P, K, RT = get_3x4_P_matrix_from_blender(cam)
    currentCameraObj = bpy.data.objects[tmp]
    scene.camera = currentCameraObj
    bpy.data.scenes["Scene"].render.filepath = "/Users/.../BlenderOutputParams/Camera"+"%.3d.png" %i
    snapshot_name = "Camera"+"%.3d.png" %i
    print(snapshot_name)
    print("RT = ", RT.round(decimals=4))
    print("P = ", P.round(decimals=4))
    bpy.ops.render.render( write_still=True )
    # write RT values in txt file
    RTmat = []
    for i in range(0,3):
        for j in range(0,4):
            RTmat.append(round(RT[i][j],6))
    RTfile.write(str(RTmat)[1 : -1]+ "\n")
    # write P values in txt file
    Pmat = []
    for i in range(0,3):
        for j in range(0,4):
            Pmat.append(round(P[i][j],6))
    Pfile.write(str(Pmat)[1 : -1]+ "\n")
RTfile.close()
Pfile.close()

print("K = ", K.astype(int))

Kmat = []
for i in range(0,3):
    for j in range(0,3):
        Kmat.append(round(K[i][j],6))
Kfile = open(txtLoc + "Kmat.txt", 'a')
Kfile.write(str(Kmat)[1 : -1]+ "\n")
Kfile.close()
