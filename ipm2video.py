import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


def main():
    data_path = './kitti_video_offset/'
    fps = 20
    size = (1002, 1002)
    video = cv2.VideoWriter("./kitti_video_offset/IPM_1.avi", cv2.VideoWriter_fourcc(*"MJPG"), fps, size)

    for i in range(50, 128):
        image_path = data_path + "{:010d}.png".format(i)
        print(image_path)
        img = cv2.imread(image_path)
        video.write(img)

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()