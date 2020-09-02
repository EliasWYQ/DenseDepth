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
        self.tt = np.array([camera_info.camera_x, camera_info.camera_y, camera_info.camera_height])[:, None]

        self.R_inv = np.array([[0.007533745, -0.9999714, -0.000616602],
                         [0.01480249, 0.0007280733, -0.9998902],
                         [0.9998621, 0.00752379, 0.01480755]])
        self.t = np.array([-0.004069766, -0.07631618, -0.2717806])[:, None]
        self.tt[2] = -np.dot(self.R_inv[:, 2], self.t)  # 4x4 translation matrix in 3d space (3d homo coordinate)
        self.tt[0] = -np.dot(self.R_inv[:, 0], self.t)
        self.tt[1] = -np.dot(self.R_inv[:, 1], self.t)
        self.R = np.linalg.inv(self.R_inv)


#calculate the offset
        self.T = np.eye(4)
        self.T[2, 3] = -1.65  # 4x4 translation matrix in 3d space (3d homo coordinate)
        self.T[0, 3] = self.t[0]
        self.T[1, 3] = self.t[1]

        #        self.K = np.array([[camera_info.f_x, 0, camera_info.u_x],
        #                           [0, camera_info.f_y, camera_info.u_y],
        #                           [0, 0, 1]]).astype(np.float)  # 3x3 intrinsic perspective projection matrix

        self.normal_c = np.dot(self.R_inv,
                               np.array([0, 0, 1])[:, None])  # normal of ground plane equation in camera coordinates
        self.const_c = np.dot(self.normal_c.T, np.dot(self.R_inv, np.dot(self.T, np.array([0, 0, 0, 1])[:, None])[
                                                                  :3]))  # constant of ground plane equation in camera coordinates

        self.u_0, self.v_0 = self.find_0()
        self.u_0 = 621
        self.v_0 = 374
        self.xyz_0 = self.uv2xy_c(np.array([self.u_0, self.v_0])[:, None])
        self.offset = self.xyz_0[2]
        self.ME_d = 0
        self.useful_n = 0
#        print
#        self.tt = np.array([camera_info.camera_x, camera_info.camera_y, camera_info.camera_height])[:, None]




#        self.K = np.array([[camera_info.f_x, 0, camera_info.u_x],
#                           [0, camera_info.f_y, camera_info.u_y],
#                           [0, 0, 1]]).astype(np.float)  # 3x3 intrinsic perspective projection matrix



#        ipm_top = self.ipm_top = max(ipm_info.top, vp[1] + ipm_info.input_height / 15)
#        uv_limits = self.uv_limits = np.array([[ipm_info.left, ipm_top],
#                                               [ipm_info.right, ipm_top],
#                                               [ipm_info.left, ipm_info.bottom],
#                                               [ipm_info.right,
#                                                ipm_info.bottom]]).T  # the limits of the area on the uv map to be IPM-converted

    def find_0(self):
        u_min = self.ipm_info.right
        v_min = self.ipm_info.bottom
        u_max = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if self.dep[v, u] == 0:
                    if u < u_min:
                        u_min = u
                    if u > u_max:
                        u_max = u
                    if v < v_min:
                        v_min = v
        u_0 = np.around((u_min + u_max) / 2)
        if v_min < self.ipm_info.bottom - 30:
            v_min = self.ipm_info.bottom
        return u_0, v_min

    def uv2xy_c(self, uvs):  # all points have z=0 (ground plane): find (x,y,z)_c first, then x_w, y_w = (R^-1 (x,y,z)_c)[:2]
        uvs = (uvs - np.array([self.camera_info.u_x, self.camera_info.u_y])[:, None]) / \
                  np.array([self.camera_info.f_x, self.camera_info.f_y])[:, None]  # converted using camara intrinsic parameters
        uvs = np.vstack((uvs, np.ones(uvs.shape[1])))
        xyz_c = (self.const_c / np.dot(self.normal_c.T, uvs)) * uvs  # solve the equation, get (x,y,z) on the ground plane in camera coordinates
        return xyz_c


    def uvz2xy_c(self, uv, depth): # z means depth


        # depth = self.offset + np.dot((60 - self.offset), depth)
        #depth = self.offset + 80*depth
        #self.dep[uv[1], uv[0]] = depth
        if gt[uv[1], uv[0]] != 0:
            self.useful_n += 1
            error = np.abs(gt[uv[1], uv[0]] - depth)
            if int(error) > 1:
                self.ME_d = self.ME_d + error
                gt[uv[1], uv[0]] = 255
        x_c = (uv[0] - self.camera_info.u_x) / self.camera_info.f_x * depth
        y_c = (uv[1] - self.camera_info.u_y) / self.camera_info.f_y * depth
        xyz_c = np.array([x_c, y_c, depth])
#        print(xyz_c)

#        c = (uv[1] - camera_info.u_y) / camera_info.f_y
#        c = c * c
#        y_c = math.sqrt(c / (1+c)) * depth
#        if uv[1] < self.camera_info.u_y:
#            y_c = - y_c
#        z_c = math.sqrt(1 / (1+c)) * depth
#        x_c = (uv[0] - camera_info.u_x) / camera_info.f_x * z_c
#        xyz_c = np.array([x_c, y_c, z_c])

#        c = (uv[0] - camera_info.u_x) / camera_info.f_x
#        c = c * c
#        x_c = math.sqrt(c / (1+c)) * depth
#        if uv[0] < self.camera_info.u_x:
#            x_c = - x_c
#        z_c = math.sqrt(1 / (1+c)) * depth
#        y_c = (uv[1] - camera_info.u_y) / camera_info.f_y * z_c
#        xyz_c = np.array([x_c, y_c, z_c])

 #       depth = depth + self.xyz_0[2]

# d^2 = x^2 + y^2 + z^2
        # c_x = (uv[0] - self.camera_info.u_x) / self.camera_info.f_x
        # c_y = (uv[1] - self.camera_info.u_y) / self.camera_info.f_y
        # z_c = math.sqrt(1/(c_x * c_x + c_y * c_y + 1)) * depth
        # x_c = c_x * z_c
        # y_c = c_y * z_c


#        if uv[0] < self.camera_info.u_x:
#            x_c = - x_c
#        if uv[1] < self.camera_info.u_y:
#            y_c = - y_c
#         xyz_c = np.array([x_c, y_c, z_c])


        # c_x = (uv[0] - camera_info.u_x) / camera_info.f_x
        # c_y = (uv[1] - camera_info.u_y) / camera_info.f_y
        # if (self.seg[uv[1], uv[0]] < 11) or (self.seg[uv[1], uv[0]] == 16):
        #     y_c = -self.camera_info.camera_y
        #     z_c = (depth * depth - y_c * y_c) / (c_x * c_x + 1)
        #     if z_c < 0:
        #         z_c = -z_c
        #     z_c = math.sqrt(z_c)
        #     x_c = c_x * z_c
        # else:
        #     z_c = math.sqrt(1 / (c_x * c_x + c_y * c_y + 1)) * depth
        #     x_c = c_x * z_c
        #     y_c = c_y * z_c
        # xyz_c = np.array([x_c, y_c, z_c])

#        xyz_w = np.dot(depth, np.dot(self.R_inv, np.dot(self.K_inv, uvs))) - np.dot(self.R_inv, self.tt)
#        xyz_w = np.around(xyz_w[0:1]).astype(np.int)

        return xyz_c

    def xy_c2xy_w(self, xyz_c):

        xyz_w = np.dot(self.R, xyz_c) + self.tt  # c = R*w + t

        return xyz_w

    def __call__(self):
        return self.ipm()

    def ipm(self):
        xy_img = np.zeros((1002, 1002, 3))
#        v_min = self.find_v_min()
        for u in range(self.ipm_info.left, self.ipm_info.right):
#            for v in range(self.ipm_info.top, self.v_0 + 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
   #             if self.dep[v, u] > 0.5:
#            for v in range(self.ipm_info.bottom - 20, self.ipm_info.bottom - 1):
                    xy_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                    xy_w = self.xy_c2xy_w(xy_c[:, None])  # xy_c[:, None]
                    y_w = int(np.around(xy_w[1, 0] * 10) + 500)
                    x_w = int(np.around(xy_w[0, 0] * 10) + 500)
                    # if (x_w < 1001) and (x_w > -1) and (y_w < 1001) and (y_w > -1):
                    #     if (self.seg[v, u] < 11) or (self.seg[v, u] == 16) :
                    #         if xy_img[y_w, x_w, 0] == 0:
                    #             xy_img[y_w, x_w, 2] = 255
                    #     else:
                    #         if (self.seg[v, u] == 12) or (self.seg[v, u] == 21):
                    #             if xy_img[y_w, x_w, 0] == 0:     # should update to compare xy_w and xmin, ymin to decide accept or not
                    #                 xy_img[y_w, x_w, 1] = 255
                    #         else:
                    #             xy_img[y_w, x_w, 0] = 255
                    #             xy_img[y_w, x_w, 1] = 0
                    #             xy_img[y_w, x_w, 2] = 0
                    if (x_w < 1001) and (x_w > -1) and (y_w < 1001) and (y_w > -1):
                        if (self.seg[v, u] == 7): #or (self.seg[v, u] == 8):
                            if xy_img[y_w, x_w, 0] == 0:
                                xy_img[y_w, x_w, 2] = 255
                        else:
                            if self.seg[v, u] == 8:
                                if xy_img[y_w, x_w, 0] == 0:     # should update to compare xy_w and xmin, ymin to decide accept or not
                                    xy_img[y_w, x_w, 1] = 255
                            else:
                                xy_img[y_w, x_w, 0] = 255
                                xy_img[y_w, x_w, 1] = 0
                                xy_img[y_w, x_w, 2] = 0
        self.ME_d = self.ME_d / self.useful_n

        return xy_img

    def ipm2(self):
        xy_img = np.zeros((1002, 1002, 3))
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
#            for v in range(self.ipm_info.bottom - 100, self.ipm_info.bottom - 1):
                if self.seg[v, u] == 1:
                    xy_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                    xy_w = self.xy_c2xy_w(xy_c[:, None])
                    y_w = int(np.around(xy_w[1, 0] * 10) + 500)
                    x_w = int(np.around(xy_w[0, 0] * 10) + 500)
                    xy_img[y_w, x_w, 1] = 255
        return xy_img


    def ipm3(self):
        x = np.zeros(300000)
        y = np.zeros(300000)
        z = np.zeros(300000)
        m = 0
        for u in range(self.ipm_info.left, self.ipm_info.right - 1):
            for v in range(self.ipm_info.top, self.ipm_info.bottom - 1):
                if self.dep[v, u] > 1:
#                    if self.seg[v, u] == 7:
                        xyz_c = self.uvz2xy_c(np.array([u, v]), self.dep[v][u])
                        x[m] = xyz_c[0]
                        y[m] = xyz_c[1]
                        z[m] = xyz_c[2]
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

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt


    camera_info = _DictObjHolder({
  #      "f_x": 309 / 5 * 8,  # focal length x
  #      "f_y": 344 / 5 * 8,  # focal length y
  #       "f_x": 850,  # focal length x 85000
  #       "f_y": 850,  # focal length y
  #       "u_x": 1023.5,  # optical center x
  #       "u_y": 251.5,  # optical center y

        "f_x": 718.856,  # focal length x 85000
        "f_y": 718.856,  # focal length y
        "u_x": 607.1928,  # optical center x
        "u_y": 185.2157,  # optical center y

        "camera_height": 0.0007959042,  # camera height in `m`
        "camera_x": 0.0004248599,
        "camera_y": -0.06286299,
        "pitch": 1.487732,  # rotation degree around y
        "roll": 0.04961946,  # rotation degree around x
        "yaw": -1.631226, # rotation degree around z

    })
    ipm_info = _DictObjHolder({

        # "input_width": 2048,
        # "input_height": 512,
        # "left": 0,
        # "right": 2048,
        # "top": 300,
        # "bottom": 512

        "input_width": 1242,
        "input_height": 375,
        "left": 0,
        "right": 1242,
        "top": 220,
        "bottom": 375
    })

    import mpl_toolkits.mplot3d as p3d
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

    import cv2 as cv

#    img = cv.imread('/home/eliaswyq/DenseDepth/ep2.png')

    img = cv.imread('/home/eliaswyq/DenseDepth/examples/kitti_3.png')

    dep = np.load('/home/eliaswyq/DenseDepth/examples/DD_dep_50.npy')
  #  dep = dep[:, :, 1]

    gt = cv.imread('kitti_video_offset/gt_0000000050.png')
    gt = gt[:, :, 1]
#    dep = np.dot(80, dep)


    # dep_flip = np.load('/home/eliaswyq/DenseDepth/dep_kitti_3_flip.npy')
    # dep_flip = dep_flip[:, :, 1]
    # dep_flip = np.dot(80, dep_flip)
    #
    # dep_flip = cv.flip(dep_flip, 1)
    #
    # for u in range(0, 639):
    #     for v in range(0, 191):
    #         if (dep[v, u] != 0) and (dep_flip[v, u] != 0):
    #             dep[v, u] = (dep[v, u] + dep_flip[v, u]) / 2
    #         else:
    #             dep[v, u] = dep[v, u] + dep_flip[v, u]

    # dep = cv.resize(dep, (2048, 512))

    #dep = cv.resize(dep, (1242, 375))

    seg = cv.imread('/home/eliaswyq/DenseDepth/examples/seg_0000000050.png')

    # seg = cv.resize(seg, (2048, 512))

#    seg = cv.resize(seg, (1241, 376))

    img = img[:, :, ::-1]

#    dep = dep[:, :, ::-1]

#    seg = seg[:, :, ::-1]

    if len(img.shape) == 3:
        img = np.dot(img, [0.299, 0.587, 0.114])

# IPM_process
    ipm = IPM(camera_info, ipm_info, dep, seg)
    out_img = ipm()

    out_img = cv.flip(out_img, 1)
    out_img = rotate(out_img, -90, None, 1.0)

    cv.imwrite("/home/eliaswyq/DenseDepth/examples/diff_50_new.png", gt)

    fig = plt.figure()
#    ax = fig.add_subplot(211)
#    ax.imshow(img)
    ax = fig.add_subplot(111)
    ax.imshow(out_img)
#    plt.savefig("./kitti_result/IPM_0000000050.png")
    cv.namedWindow('image')
    out_img = out_img[:, :, ::-1]
    cv.imshow('image', out_img)
#    cv.imwrite("./kitti_result/IPM_cv_0000000050.png", out_img)


#cloud points
    # ipm = IPM(camera_info, ipm_info, dep, seg)
    # x, y, z = ipm()
    #
    #
    # fig = plt.figure()
    # #    ax = fig.add_subplot(211)
    # #    ax.imshow(img)
    # ax = p3d.Axes3D(fig)
    # ax.scatter(x, y, z, c='b', s=10, alpha=0.05)
    # plt.show()
    # plt.savefig("./IPM_problem_3D_uvdis_xyz.png")