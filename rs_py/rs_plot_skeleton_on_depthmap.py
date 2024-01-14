import os
import cv2
import numpy as np
import sys
from tqdm import tqdm

from rs_py.utility import read_calib_file
from rs_py.utility import read_depth_file
from rs_py.utility import read_skeleton_file
from rs_py.utility import iterate_over_depth_skeleton

 # {0,  "Nose"},
 # {1,  "Neck"},
 # {2,  "RShoulder"},
 # {3,  "RElbow"},
 # {4,  "RWrist"},
 # {5,  "LShoulder"},
 # {6,  "LElbow"},
 # {7,  "LWrist"},
 # {8,  "MidHip"},
 # {9,  "RHip"},
 # {10, "RKnee"},
 # {11, "RAnkle"},
 # {12, "LHip"},
 # {13, "LKnee"},
 # {14, "LAnkle"},
 # {15, "REye"},
 # {16, "LEye"},
 # {17, "REar"},
 # {18, "LEar"},
 # {19, "LBigToe"},
 # {20, "LSmallToe"},
 # {21, "LHeel"},
 # {22, "RBigToe"},
 # {23, "RSmallToe"},
 # {24, "RHeel"},
 # {25, "Background"}
bone_list = [[0,1], [1,8],
             [1,2], [1,5], [2,3], [5,6], [3,4], [6,7],
             [8,9], [8,12], [9,10], [12,13], [10,11], [13,14],
             [0,15], [0,16], [15,17], [16,18],
             [11,22], [11,24], [22,23], [14,19], [14,21], [19,20]]
# color_list = ['tomato', 'r',
#               'g', 'orangered', 'limegreen', 'darkorange', 'lime', 'orange',
#               'b', 'darkviolet','royalblue', 'fuchsia', 'cornflowerblue', 'violet',
#               'g', 'g', 'orangered', 'orangered',
#               'cornflowerblue', 'cornflowerblue', 'cornflowerblue', 'violet', 'violet', 'violet'
#               ]
color_list = [(255,99,71), (255,0,0),
              (0,255,0), (255,69,0), (50,205,50), (255,140,0), (0,255,0), (255,165,0),
              (0,0,255), (148,0,211), (65,105,225), (255,0,255), (100,149,237), (238,130,238),
              (0,255,0), (0,255,0), (255,69,0), (255,69,0),
              (100,149,237), (100,149,237), (100,149,237), (238,130,238), (238,130,238), (238,130,238)
              ]

skel_thres = 0.25
def draw_skeletons(img, joints, bone_list=bone_list, color_list=color_list):
    for skel_i in range(len(joints)//25):
        joints_i = joints[skel_i*25 : (skel_i+1)*25]
        for i, pair in enumerate(bone_list):
            x1, y1, c1 = joints_i[pair[0]]
            x2, y2, c2 = joints_i[pair[1]]
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            if c1 > 0.0:
                cv2.circle(img, (x1, y1), radius=4, color=(255, 128, 0), thickness=-1)
            if c2 > 0.0:
                cv2.circle(img, (x2, y2), radius=4, color=(255, 128, 0), thickness=-1)
            if c1 > 0.0 and c2 > 0.0:
                cv2.line(img, (x1, y1), (x2, y2), color=color_list[i], thickness=2)
    return img


skeleton_prev = None
def data_process_fn(**kwargs):

    depth_files = kwargs['depth_files']
    depth_ts_idxs = kwargs['depth_ts_idxs']
    skeleton_files = kwargs['skeleton_files']
    skeleton_ts_idxs = kwargs['skeleton_ts_idxs']

    sync_ts = kwargs['sync_ts']
    scale = kwargs['scale']
    fps = kwargs['fps']

    imgs = []
    num_dev = len(depth_files)
    for dev_idx in range(num_dev):

        # (ts, file, calib)
        depth_dc = depth_files[dev_idx][depth_ts_idxs[dev_idx]]
        skeleton_dc = skeleton_files[dev_idx][skeleton_ts_idxs[dev_idx]]

        calib_data = read_calib_file(depth_dc.calib_file)

        h_d = calib_data['depth']['height']
        w_d = calib_data['depth']['width']

        depth = read_depth_file(depth_dc.file)
        skeleton = read_skeleton_file(skeleton_dc.file)

        ###############################
        # save only imgs with skeleton
        if skeleton.size == 0:
            return

        # print(f'CHECKSUM-SKELETON: {sum(skeleton.flatten())}')

        depth = depth[-h_d*w_d:].reshape(h_d, w_d)
        # camera is upside-down!!!
        depth = np.rot90(depth, 2)
        # depth_i = depth[-(h_d//3)*(w_d//3+2):].reshape(h_d//3, w_d//3+2)
        # depth = cv2.resize(depth_i, (w_d, h_d))

        depth = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03),
            cv2.COLORMAP_JET
        )

        cv2.putText(depth,
                    f"{depth_dc.device_sn} - {depth_dc.timestamp}",
                    (10, 20),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA)

        joints = []

        if len(skeleton.shape) > 1:
            max_score = max(skeleton[:,0])
            if max_score < skel_thres:
                print(f'Max score under threshold: {max_score}')
                return
            for sub in skeleton:
                score = sub[0]
                if score > skel_thres:
                    for kp_i in range(int(len(sub) // 3)):
                        x = sub[3 * kp_i + 1]
                        y = sub[3 * kp_i + 2]
                        score_i = sub[3 * kp_i + 3]
                        joints.append([x, y, score_i])

            output = draw_skeletons(depth, joints)
        elif len(skeleton.shape) == 1 and skeleton.size != 0:
            score = skeleton[0]
            if score > skel_thres:
                for kp_i in range(int(len(skeleton) // 3)):
                    x = skeleton[3 * kp_i + 1]
                    y = skeleton[3 * kp_i + 2]
                    score_i = skeleton[3 * kp_i + 3]
                    joints.append([x, y, score_i])
                output = draw_skeletons(depth, joints)
            else:
                print(f'Score under threshold: {score}')
                return

        if 'output' not in locals():
            # output = depth
            return
        new_file = depth_dc.file.replace('/depth/', '/depth_skeleton/')
        os.makedirs(os.path.dirname(new_file), exist_ok=True)
        new_file = os.path.splitext(new_file)[0] + '.png'
        cv2.imwrite(new_file, output)


    # output = np.hstack(imgs)
    # output = cv2.resize(output, (int(output.shape[1]*scale),
    #                              int(output.shape[0]*scale)))

    # name = f'output - sync={sync_ts}'
    # cv2.namedWindow(name)
    # cv2.moveWindow(name, 0, 0)
    # cv2.imshow(name, output)
    # # cv2.waitKey(1000//fps)
    # # cv2.waitKey(0)
    # cv2.waitKey(100)


if __name__ == "__main__":

    # PATH = sys.argv[1]
    # SYNC = int(sys.argv[2])
    # FPS = int(sys.argv[3])
    # SCALE = float(sys.argv[4])

    # PATH = '/code/realsense-simple-wrapper/data/local/realsense-15fps'
    # PATH = '/home/dhm/DigitalICU/ICU-Suite/test_skeleton2-15fps'

    data_path = '/home/dhm/DigitalICU/Experiments/ICUActivityProtocolling/data/skel_depth'
    for folder in tqdm(os.listdir(data_path)[2:]):
        record_name = folder
        print(folder)
        # PATH = f'/home/dhm/DigitalICU/icu_recording/{record_name}-15fps'
        PATH = os.path.join(data_path, folder)
        SYNC = 0
        FPS = 1
        SCALE = 0.75

        iterate_over_depth_skeleton(PATH, SYNC, FPS, SCALE,
                              data_process_fn=data_process_fn)

        # # generate video from image sequences
        # img_dict = {}
        # dev_list = [folder for folder in os.listdir(PATH) if os.path.isdir(os.path.join(PATH, folder))]
        # for dev in dev_list:
        #     trial_list = [folder for folder in os.listdir(os.path.join(PATH, dev)) if os.path.isdir(os.path.join(PATH, dev, folder))]
        #     img_dict[dev] = {}
        #     for trial in trial_list:
        #         if os.path.exists(os.path.join(PATH, dev, trial, 'depth_skeleton')):
        #             img_dict[dev][trial] = os.listdir(os.path.join(PATH, dev, trial, 'depth_skeleton'))
        #         else:
        #             img_dict[dev][trial] = None

        # video_depth = cv2.VideoWriter(os.path.join(PATH, f'{record_name}_depth.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 15, (3 * 848, 480))
        # # video_rpg = cv2.VideoWriter(os.path.join(PATH, f'{record_name}_rgb.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 15, (3 * 848, 480))
        # for trial in trial_list:
        #     # imgs = []
        #     depths = []
        #     for dev in dev_list:
        #         if img_dict[dev][trial] is None:
        #             continue
        #         # img_path = os.path.join(PATH, dev, trial, 'color_png', img_dict[dev][trial][i])
        #         # img = cv2.imread(img_path)
        #         # img = np.rot90(img, 2)
        #         # imgs.append(img)
        #         for i in range(len(img_dict[dev][trial])):
        #             depth_path = os.path.join(PATH, dev, trial, 'depth_skeleton', img_dict[dev][trial][i])
        #             depth = cv2.imread(depth_path)
        #             # depth = np.rot90(depth, 2)
        #             depths.append(depth)
        #         # if ('h_img' not in locals()) or ('w_img' not in locals()):
        #         #     h_img, w_img, _ = img.shape
        #
        #     if len(depths) == 0:
        #         continue
        #     depth_concat = np.concatenate(depths, axis=1)
        #     # img_concat = np.concatenate(imgs, axis=1)
        #     # video_rpg.write(img_concat)
        #     video_depth.write(depth_concat)
        #
        # # video_rpg.release()
        # video_depth.release()




