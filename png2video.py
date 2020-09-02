import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


def main():
    data_path = './raw_data/data/'
    fps = 20
    size = (1242, 375)
    video = cv2.VideoWriter("./kitti_video_offset/raw_data_1.avi", cv2.VideoWriter_fourcc(*"MJPG"), fps, size)

    for i in range(50, 129):
        image_path = data_path + "{:010d}.png".format(i)
        print(image_path)
        img = cv2.imread(image_path)
        video.write(img)

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()