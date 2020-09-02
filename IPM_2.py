# -*- coding: utf-8 -*-
import numpy as np
import math


class IPM(object):

    def __init__(self, camera_info, ipm_info, dep, seg):
        self.camera_info = camera_info
        self.ipm_info = ipm_info
        self.dep = dep
        self.seg = seg[:, :, 1]

#        for v in range(self.ipm_info.bottom - 100, self.ipm_info.bottom - 1):
#            for u in range(self.ipm_info.left, self.ipm_info.right - 1):
#                self.dep[v, u] = 10
#                self.seg[v, u] = 1

        ## Construct matrices tt, R， K
        _cy = np.cos(camera_info.yaw)
        _sy = np.sin(camera_info.yaw)
        _cp = np.cos(camera_info.pitch)
        _sp = np.sin(camera_info.pitch)
        _cr = np.cos(camera_info.roll)
        _sr = np.sin(camera_info.roll)
        tpitch = np.array([[_cp, 0, -_sp],
                         [0, 1, 0],
                         [_sp, 0, _cp]])

        troll = np.array([[1, 0, 0],
                           [0, _cr, -_sr],
                           [0, _sr, _cr]])

        tyaw = np.array([[_cy, -_sy, 0],
                               [_sy, _cy, 0],
                               [0, 0, 1]])

  #      self.R = np.dot(np.dot(tyaw, tpitch), troll)  # 3x3 Rotation matrix in 3d space
        self.R = np.dot(np.dot(troll, tpitch), tyaw)
        self.R_inv = np.linalg.inv(self.R)

        self.tt = np.array([camera_info.camera_x, camera_info.camera_y, camera_info.camera_height])[:, None]
        self.t = np.array([camera_info.camera_x, camera_info.camera_y, -1.73])[:, None]
        self.T = np.eye(4)
        self.T[2, 3] = -np.dot(self.R[:, 2], self.t)  # 4x4 translation matrix in 3d space (3d homo coordinate)
        self.T[0, 3] = -np.dot(self.R[:, 0], self.t)
        self.T[1, 3] = -np.dot(self.R[:, 1], self.t)


#        self.K = np.array([[camera_info.f_x, 0, camera_info.u_x],
#                           [0, camera_info.f_y, camera_info.u_y],
#                           [0, 0, 1]]).astype(np.float)  # 3x3 intrinsic perspective projection matrix

        self.normal_c = np.dot(self.R_inv, np.array([0,0,1])[:, None]) # normal of ground plane equation in camera coordinates
        self.const_c = np.dot(self.normal_c.T, np.dot(self.R_inv, np.dot(self.T, np.array([0,0,0,1])[:, None])[:3])) # constant of ground plane equation in camera coordinates

        u_0, v_0 = self.find_0()
        self.xyz_0 = self.uv2xy(np.array([u_0, v_0])[:, None])
        # self.xy_0 = self.uv2xy(np.array([1858, 500])[:, None])
        # u_0, v_0 = self.find_0()
        # xyz_0 = self.uv2xy(np.array([u_0, v_0])[:, None])
        # self.dep = self.dep + xyz_0[2]
        # print

#        ipm_top = self.ipm_top = max(ipm_info.top, vp[1] + ipm_info.input_height / 15)
#        uv_limits = self.uv_limits = np.array([[ipm_info.left, ipm_top],
#                                               [ipm_info.right, ipm_top],
#                                               [ipm_info.left, ipm_info.bottom],
#                                               [ipm_info.right,
#                                                ipm_info.bottom]]).T  # the limits of the area on the uv map to be IPM-converted






    def uv2xy(self, uvs):  # all points have z=0 (ground plane): find (x,y,z)_c first, then x_w, y_w = (R^-1 (x,y,z)_c)[:2]
        uvs = (uvs - np.array([self.camera_info.u_x, self.camera_info.u_y])[:, None]) / \
                  np.array([self.camera_info.f_x, self.camera_info.f_y])[:, None]  # converted using camara intrinsic parameters
        uvs = np.vstack((uvs, np.ones(uvs.shape[1])))
        xyz_c = (self.const_c / np.dot(self.normal_c.T, uvs)) * uvs  # solve the equation, get (x,y,z) on the ground plane in camera coordinates
        return xyz_c

    def find_0(self):
        u_min = self.ipm_info.right
        v_min = self.ipm_info.bottom
        u_max = 0
        v_max = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if self.dep[v, u] == 0:
                    if u < u_min:
                        u_min = u
                    if u > u_max:
                        u_max = u
                    if v < v_min:
                        v_min = v
                    if v > v_max:
                        v_max = v
        u_0 = np.around((u_min + u_max) / 2)
        v_0 = np.around((v_min + v_max) / 2)
        return u_0, v_0

        u_min = self.ipm_info.right
        # v_min = self.ipm_info.bottom
        u_max = 0
        v_max = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if self.dep[v, u] == 0:
                    if u < u_min:
                        u_min = u
                    if u > u_max:
                        u_max = u
                    if v < v_min:
                        v_min = v
                    if v > v_max:
                        v_max = v
        u_0 = np.around((u_min + u_max) / 2)
        # v_0 = np.around((v_min + v_max) / 2)
        return u_0, v_min





    def uvz2xy_c(self, uv, depth): # z means depth


        # c_x = (uv[0] - camera_info.u_x) / camera_info.f_x
        # c_y = (uv[1] - camera_info.u_y) / camera_info.f_y
        # # z_c = math.sqrt(1/(c_x * c_x + c_y * c_y + 1)) * depth
        # # x_c = c_x * z_c
        # # y_c = c_y * z_c
        # #
        # # xyz_c = np.array([x_c, y_c, z_c])
        #
        #
        #
        # a = c_x * c_x + c_y * c_y + 1
        # b = -2 * (c_x * self.xyz_0[0] + c_y * self.xyz_0[1] + self.xyz_0[2])
        # c = self.xyz_0[0] * self.xyz_0[0] + self.xyz_0[1] * self.xyz_0[1] + self.xyz_0[2] * self.xyz_0[2] - depth * depth
        # if b*b - 4*a*c < 0:
        #     m = 0
        # else:
        #     m = math.sqrt(b*b - 4*a*c)
        # z_c = (-b + m) / (2 * a)
        # x_c = c_x * z_c
        # y_c = c_y * z_c
        # xyz_c = np.array([x_c, y_c, z_c])

        x_c = (uv[0] - self.camera_info.u_x) / self.camera_info.f_x * depth
        y_c = (uv[1] - self.camera_info.u_y) / self.camera_info.f_y * depth
        xyz_c = np.array([x_c, y_c, depth])

        return xyz_c

    def xy_c2xy_w(self, xyz_c):

        xyz_w = np.dot(self.R, xyz_c) + self.tt  # c = R*w + t

        return xyz_w

    def __call__(self):
        return self.ipm()

    def ipm(self):
        xy_img = np.zeros((1002, 1002, 3))
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
#            for v in range(self.ipm_info.bottom - 20, self.ipm_info.bottom - 1):
                xy_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                xy_w = self.xy_c2xy_w(xy_c[:, None])
                #xy_w = self.xy_c2xy_w(xy_c)
                y_w = int(np.around(xy_w[1, 0] * 10) + 500)
                x_w = int(np.around(xy_w[0, 0] * 10) + 500)
                if (x_w < 1001) and (x_w > -1) and (y_w < 1001) and (y_w > -1):
                    if (self.seg[v, u] < 11) or (self.seg[v, u] == 16) :
                        if xy_img[y_w, x_w, 0] == 0:
                            xy_img[y_w, x_w, 2] = 255
                    else:
                        if (self.seg[v, u] == 12) or (self.seg[v, u] == 21):
                            if xy_img[y_w, x_w, 0] == 0:     # should update to compare xy_w and xmin, ymin to decide accept or not
                                xy_img[y_w, x_w, 1] = 255
                        else:
                            xy_img[y_w, x_w, 0] = 255
                            xy_img[y_w, x_w, 1] = 0
                            xy_img[y_w, x_w, 2] = 0
        return xy_img

    def ipm2(self):
        x = np.zeros(300000)
        y = np.zeros(300000)
        z = np.zeros(300000)
        m = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if (self.seg[v, u] < 11) or (self.seg[v, u] == 16):
                    xyz_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                    x[m] = xyz_c[0]
                    y[m] = xyz_c[1]
                    z[m] = xyz_c[2]
                    m = m + 1
        return x, y, z

    def ipm3(self):
        uu = np.zeros(400000)
        vv = np.zeros(400000)
        z = np.zeros(400000)
        m = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if (self.seg[v, u] < 11) or (self.seg[v, u] == 16):
#                    xyz_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                    uu[m] = u
                    vv[m] = v
                    z[m] = self.dep[v, u]
#                    z[m] = xyz_c[2]
                    m = m + 1
        return uu, vv, z

    def ipm4(self):
        x = np.zeros(600000)
        y = np.zeros(600000)
        z = np.zeros(600000)
        m = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if (self.seg[v, u] < 11) or (self.seg[v, u] == 16):
                    xyz_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                    xyz_w = self.xy_c2xy_w(xyz_c[:, None])
                    x[m] = xyz_w[0]
                    y[m] = xyz_w[1]
                    z[m] = xyz_w[2]
                    m = m + 1
        return x, y, z


class _DictObjHolder(object):
    def __init__(self, dct):
        self.dct = dct

    def __getattr__(self, name):
        return self.dct[name]

def rotate(image, angle, center = None, scale = 1.0):

    (h, w) = image.shape[:2]

    if center is None:
        center = (w / 2, h / 2)

    M = cv.getRotationMatrix2D(center, angle, scale)
    rotated = cv.warpAffine(image, M, (w, h))

    return rotated

if __name__ == "__main__":
    import os
    import sys

  #  sys.path.insert(0, os.path.expanduser("~/caffe/python/"))

    import matplotlib
    import mpl_toolkits.mplot3d as p3d

#    matplotlib.use("Agg")
    import matplotlib.pyplot as plt


    camera_info = _DictObjHolder({
  #      "f_x": 309 / 5 * 8,  # focal length x
  #      "f_y": 344 / 5 * 8,  # focal length y
        "f_x": 850,  # focal length x 85000
        "f_y": 850,  # focal length y
        "u_x": 1023.5,  # optical center x
        "u_y": 251.5,  # optical center y
        "camera_height": 0.0007959042,  # camera height in `m`
        "camera_x": 0.0004248599,
        "camera_y": -0.06286299,
        "pitch": 1.487732,  # rotation degree around y
        "roll": 0.04961946,  # rotation degree around x
        "yaw": -1.631226, # rotation degree around z

    })
    ipm_info = _DictObjHolder({

        "input_width": 2048,
        "input_height": 512,
        "left": 0,
        "right": 2048,
        "top": 300,
        "bottom": 512
    })

    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

    import cv2 as cv

    img = cv.imread('/home/eliaswyq/DenseDepth/examples/a.png')
#
#     img = cv.imread('/home/eliaswyq/DenseDepth/examples/0000000050.png')
#
# #    img = np.uint8(np.clip((1.5 * img + 10), 0, 255))
#
#     img = cv.resize(img, (1280, 384))
#
#     cv.imwrite("/home/eliaswyq/DenseDepth/examples/0000000050.png", img)
#     #
#     img_flip_along_y = cv.flip(img, 1)  # 把围绕Y轴翻转的图像存进img_flip_along_y
#     #
#     cv.imwrite("/home/eliaswyq/DenseDepth/examples/kitti_3_flip.png", img_flip_along_y)  # 显示img_flip_along_y图像

    dep = np.load('/home/eliaswyq/DenseDepth/examples/dep_1.npy')

    # dep = dep[:, :, 1]
    #
    # dep = np.dot(80, dep)
    #
    dep = cv.resize(dep, (2048, 512))

    seg = cv.imread('/home/eliaswyq/DenseDepth/seg.png')

    seg = cv.resize(seg, (2048, 512))

    img = img[:, :, ::-1]

#    dep = dep[:, :, ::-1]

    seg = seg[:, :, ::-1]

    if len(img.shape) == 3:
        img = np.dot(img, [0.299, 0.587, 0.114])
    ipm = IPM(camera_info, ipm_info, dep, seg)
#    x, y, z = ipm()
    out_img = ipm()

    # out_img = cv.flip(out_img, 1)
    out_img = rotate(out_img, -90, None, 1.0)

#    out_img = rotate(out_img, -90, None, 1.0)

    fig = plt.figure()
#    ax = fig.add_subplot(211)
#    ax.imshow(img)
#     ax = p3d.Axes3D(fig)
#     ax.scatter(x, y, z, c='b', s=10, alpha=0.05)
#     plt.show()
#     plt.savefig("./IPM_problem_3D_uvdis_xyz.png")
    cv.namedWindow('image')
    cv.imshow('image', out_img)
    cv.imwrite("examples/1_new.png", out_img)