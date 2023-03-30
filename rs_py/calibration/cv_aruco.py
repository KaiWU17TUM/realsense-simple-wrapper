import argparse
import cv2
import json
import numpy as np
import os

from typing import Tuple, Optional, Union

from rs_py.wrapper.rs_wrapper import RealsenseWrapper
from rs_py.wrapper.rs_wrapper import get_rs_parser

from rs_py.utility import printout
from rs_py.utility import str2bool


THICKNESS = 2


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument('--ar-only',
                        type=str2bool,
                        default=False,
                        help='whether to use aruco code only without rsw.')
    parser.add_argument('--ar-mode',
                        type=str,
                        # required=True,
                        # default='generate',
                        # default='detect',
                        default='estimatepose',
                        help="run code to generate or detect aruco markers.")
    parser.add_argument('--ar-type',
                        type=str,
                        default="DICT_4X4_50",
                        help="type of ArUCo tag to generate")
    parser.add_argument('--ar-display',
                        type=int,
                        default=0,
                        help="1: generated ArUCo tag"
                             "2: detected ArUCo tag"
                             "3: pose estimation"
                             "4: detection and pose estimation"
                             "12: detected ArUCo tag with rsw"
                             "13: pose estimation with rsw"
                             "14: detection and pose estimation with rsw")
    parser.add_argument('--ar-save-path',
                        type=str,
                        default='-1',
                        help="path to save image containing ArUCo tag")

    # generate
    parser.add_argument('--ar-id',
                        type=int,
                        default=1,
                        help="ID of ArUCo tag to generate")
    parser.add_argument('--ar-size',
                        type=int,
                        default="600",
                        help="size of ArUCo tag to generate")
    parser.add_argument('--ar-dummy',
                        type=int,
                        default=0,
                        help="if 1, generate a dummy tag with white border")
    # detect
    parser.add_argument('--ar-image-path',
                        type=str,
                        default='-1',
                        # default='output/aruco/dummy.png',
                        help="size of ArUCo tag to generate")
    parser.add_argument('--ar-include-rejected',
                        type=int,
                        default=0,
                        help="if 1, include rejected points in the "
                             "detected results")
    # pose
    parser.add_argument('--ar-calib-path',
                        type=str,
                        default='-1',
                        help="Path to json file with calibration matrix "
                             "and distortion coefficients")
    # # RSW
    # parser.add_argument('--ar-with-rsw',
    #                     type=str2bool,
    #                     default=False,
    #                     help="whether to use aruco code with rsw. If True, "
    #                          "it ignores --ar-calib-path, --ar-save-path")
    return parser


class ArucoWrapper:
    # main reference :
    # https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

    def __init__(self,
                 args: argparse.Namespace,
                 rsw: Optional[RealsenseWrapper] = None) -> None:
        # 1. general
        self.mode = args.ar_mode
        self.type = args.ar_type
        self.display = args.ar_display
        self.save_path = None
        self.dev_sn_list = []
        self.save_path_per_dev = {}
        # 2. generation
        self.id = args.ar_id
        self.size = args.ar_size
        self.dummy = args.ar_dummy
        # 3. detection
        self.image_path = args.ar_image_path
        self.include_rejected = args.ar_include_rejected
        # 4. pose estimation
        self.calib_path = None
        self.intrinsic_mat = None
        self.coeffs = None
        self.calib_path_per_dev = {}
        self.intrinsic_mat_per_dev = {}
        self.coeffs_per_dev = {}
        # 5. whether to use with rsw
        if rsw is None:
            # 5.1. path to save results
            self.save_path = args.ar_save_path
            if self.save_path == '-1':
                self.save_path = None
            else:
                os.makedirs(self.save_path, exist_ok=True)
            # 5.2. calib for pose estimation
            self.calib_path = args.ar_calib_path
            if self.calib_path != '-1':
                with open(self.calib_path) as calib_file:
                    self.calib_data = json.load(calib_file)
                self.intrinsic_mat, self.coeffs = \
                    self.get_camera_matrix_and_distortion_coeffs(
                        self.calib_data)
        else:
            self.dev_sn_list = [sn for sn, _ in rsw.available_devices]
            for dev_sn in self.dev_sn_list:
                # 5.1. path to save results
                save_path = rsw.storage_paths_per_dev[dev_sn].color
                save_path = save_path.replace('/color',
                                              f'/color_{ar_args.ar_mode}')
                os.makedirs(save_path, exist_ok=True)
                self.save_path_per_dev[dev_sn] = save_path
                # 5.2. calib for pose estimation
                calib_path = rsw.storage_paths_per_dev[dev_sn].calib
                calib_path = os.path.join(calib_path,
                                          os.listdir(calib_path)[0])
                self.calib_path_per_dev[dev_sn] = calib_path
                with open(calib_path) as calib_file:
                    calib_data = json.load(calib_file)
                intrinsic_mat, coeffs = \
                    self.get_camera_matrix_and_distortion_coeffs(
                        calib_data)
                self.intrinsic_mat_per_dev[dev_sn] = intrinsic_mat
                self.coeffs_per_dev[dev_sn] = coeffs

    def step(self,
             image: Optional[Union[np.ndarray, dict]] = None,
             idx: Union[int, dict] = 0):

        if self.mode == 'generate':
            return self.generate_marker()

        elif self.mode == 'detect':
            if len(self.dev_sn_list) > 0:
                output = {}
                for dev_sn in self.dev_sn_list:
                    self.save_path = self.save_path_per_dev[dev_sn]
                    output[dev_sn] = self.detect_markers(image[dev_sn],
                                                         idx[dev_sn])
                if self.display == 12:
                    images = np.hstack([i['image'] for _, i in output.items()])
                    name = f"ArUCo Tag, {self.type}, {self.id}, {self.dev_sn_list}"  # noqa
                    cv2.namedWindow(name)
                    # cv2.moveWindow(name, 0, 0)
                    images = cv2.resize(images, (images.shape[1]//2,
                                                 images.shape[0]//2))
                    cv2.imshow(name, images)
                    cv2.waitKey(0)
                return output

            else:
                return self.detect_markers(image, idx)

        elif self.mode == 'estimatepose':
            if len(self.dev_sn_list) > 0:
                output = {}
                for dev_sn in self.dev_sn_list:
                    self.save_path = self.save_path_per_dev[dev_sn]
                    self.calib_path = self.calib_path_per_dev[dev_sn]
                    self.intrinsic_mat = self.intrinsic_mat_per_dev[dev_sn]
                    self.coeffs = self.coeffs_per_dev[dev_sn]
                    output[dev_sn] = self.estimate_pose(image[dev_sn],
                                                        idx[dev_sn])
                if self.display in [13, 14]:
                    images = np.hstack([i['image'] for _, i in output.items()])
                    name = f"ArUCo Tag, {self.type}, {self.id}, {self.dev_sn_list}"  # noqa
                    cv2.namedWindow(name)
                    # cv2.moveWindow(name, 0, 0)
                    images = cv2.resize(images, (images.shape[1]//2,
                                                 images.shape[0]//2))
                    cv2.imshow(name, images)
                    cv2.waitKey(0)
                return output

            else:
                return self.estimate_pose(image, idx)

        else:
            raise ValueError("Unknown mode...")

    # Based on :
    # https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
    def generate_marker(self):

        printout(f"generating ArUCo type {self.type} with ID {self.id}", 'i')

        aruco_dict = self.get_aruco_dict()
        marker = np.zeros((self.size, self.size, 1), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dict, self.id, self.size, marker, 1)

        if self.save_path is not None:
            if self.dummy == 1:
                output_file = os.path.join(self.save_path, f'dummy.png')
                dummy = np.ones((self.size+200, self.size+200, 1),
                                dtype=np.uint8) * 255
                dummy[100:-100, 100:-100, :] = marker
                cv2.imwrite(output_file, dummy)
            else:
                output_file = f'aruco_{self.type}_{self.id}.png'
                output_file = os.path.join(self.save_path, output_file)
                cv2.imwrite(output_file, marker)

        if self.display == 1:
            name = "ArUCo Tag"
            cv2.namedWindow(name)
            # cv2.moveWindow(name, 0, 0)
            cv2.imshow(name, marker)
            cv2.waitKey(0)

        printout(f"generated ArUCo type {self.type} with ID {self.id}", 'i')

    # Based on :
    # https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
    def detect_markers(self,
                       image: Optional[np.ndarray] = None,
                       idx: int = 0
                       ) -> dict:

        if image is None:
            printout(f"detecting ArUCo from image : {self.image_path}", 'i')
            image = cv2.imread(self.image_path)
        else:
            printout(f"detecting ArUCo from live image", 'i')

        printout(f"detecting ArUCo type {self.type} with ID {self.id}", 'i')

        aruco_dict = self.get_aruco_dict()
        aruco_params = self.get_aruco_params()
        corners, ids, rejected = cv2.aruco.detectMarkers(
            image, aruco_dict, parameters=aruco_params)

        if self.display in [2, 4] or self.save_path is not None:

            # Option 1 ----------
            # # Draw a square around the markers
            # cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))

            # Option 2 ----------
            # verify *at least* one ArUco marker was detected
            if ids is not None:
                if len(ids) > 0:
                    # flatten the ArUco IDs list
                    ids = ids.flatten()
                    # loop over the detected ArUCo corners
                    for (marker_corner, marker_id) in zip(corners, ids):
                        # extract the marker corners (which are always
                        # returned in tl, tr, br, and bl order)
                        marker_corner = marker_corner.reshape((4, 2))
                        (tl, tr, br, bl) = marker_corner
                        # convert each of the (x, y)-coordinate pairs to ints
                        tr = (int(tr[0]), int(tr[1]))
                        br = (int(br[0]), int(br[1]))
                        bl = (int(bl[0]), int(bl[1]))
                        tl = (int(tl[0]), int(tl[1]))
                        # draw the bounding box of the ArUCo detection
                        cv2.line(image, tl, tr, (0, 255, 0), 2)
                        cv2.line(image, tr, br, (0, 255, 0), 2)
                        cv2.line(image, br, bl, (0, 255, 0), 2)
                        cv2.line(image, bl, tl, (0, 255, 0), 2)
                        # compute and draw the center (x, y)-coordinates
                        cX = int((tl[0] + br[0]) / 2.0)
                        cY = int((tl[1] + br[1]) / 2.0)
                        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                        # draw the ArUco marker ID on the image
                        cv2.putText(image, str(marker_id),
                                    (cX+10, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 0), 2)
                        printout(f"ArUco marker ID {marker_id}", 'i')

            if self.include_rejected == 1:
                printout(f"rejected points : {rejected}", 'i')
                for i in rejected:
                    for points in i:
                        for point in points:
                            cv2.circle(image,
                                       (int(point[0]), int(point[1])),
                                       4, (255, 0, 255), -1)

            if self.save_path is not None:
                output_file = f'aruco_{self.type}_{self.id}_detected_{idx}.png'
                output_file = os.path.join(self.save_path, output_file)
                cv2.imwrite(output_file, image)

            if self.display == 2:
                name = f"ArUCo Tag, {self.type}, {self.id}, {self.dev_sn}"
                cv2.namedWindow(name)
                # cv2.moveWindow(name, 0, 0)
                cv2.imshow(name, image)
                cv2.waitKey(0)

        return {
            'image': image,
            'corners': corners,
            'ids': ids,
            'rejected': rejected
        }

    # based on:
    # https://github.com/ddelago/Aruco-Marker-Calibration-and-Pose-Estimation/blob/master/pose_marker.py
    def estimate_pose(self,
                      image: Optional[np.ndarray] = None,
                      idx: int = 0
                      ):

        def _drawCube(img, corners, imgpts):
            imgpts = np.int32(imgpts).reshape(-1, 2)
            # draw ground floor in green
            # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
            # draw pillars in blue color
            for i, j in zip(range(4), range(4, 8)):
                img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255),
                               THICKNESS)
            # draw top layer in red color
            img = cv2.drawContours(
                img, [imgpts[4:]], -1, (0, 0, 255), THICKNESS)
            return img

        # assert hasattr(self, 'calib_data')

        output_dict = self.detect_markers(image, idx)
        image = output_dict['image']
        corners = output_dict['corners']
        ids = output_dict['ids']
        rejected = output_dict['rejected']

        if ids is None:
            return {
                'image': image,
                'corners': corners,
                'ids': ids,
                'rejected': rejected,
                'rvecs': None,
                'tvecs': None,
                '_objPoints': None,
            }

        if len(ids) == 0:
            return {
                'image': image,
                'corners': corners,
                'ids': ids,
                'rejected': rejected,
                'rvecs': None,
                'tvecs': None,
                '_objPoints': None,
            }

        # Estimate the posture per each Aruco marker
        rotation_vectors, translation_vectors, obj_points = \
            cv2.aruco.estimatePoseSingleMarkers(
                corners,
                markerLength=0.02,
                cameraMatrix=self.intrinsic_mat,
                distCoeffs=self.coeffs
            )

        if self.display in [3, 4] or self.save_path is not None:

            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                # CUBE
                objectPoints = np.float32([
                    [-.5, -.5, 0], [-.5, .5, 0],
                    [.5, .5, 0], [.5, -.5, 0],
                    [-.5, -.5, 1], [-.5, .5, 1],
                    [.5, .5, 1], [.5, -.5, 1]]) * 0.05
                imgpts, jac = cv2.projectPoints(objectPoints=objectPoints,
                                                rvec=rvec,
                                                tvec=tvec,
                                                cameraMatrix=self.intrinsic_mat,
                                                distCoeffs=self.coeffs)
                image = _drawCube(image, corners, imgpts)
                # AXES
                # image = cv2.drawFrameAxes(image=image,
                #                           cameraMatrix=self.intrinsic_mat,
                #                           distCoeffs=self.coeffs,
                #                           rvec=rvec,
                #                           tvec=tvec,
                #                           length=0.1,
                #                           thickness=THICKNESS)

            if self.save_path is not None:
                output_file = f'aruco_{self.type}_{self.id}_pose_{idx}.png'
                output_file = os.path.join(self.save_path, output_file)
                cv2.imwrite(output_file, image)

            if self.display in [3, 4]:
                name = f"ArUCo Tag, {self.type}, {self.id}, {self.dev_sn}"
                cv2.namedWindow(name)
                # cv2.moveWindow(name, 0, 0)
                cv2.imshow(name, image)
                cv2.waitKey(0)

        return {
            'image': image,
            'corners': corners,
            'ids': ids,
            'rejected': rejected,
            'rvecs': rotation_vectors,
            'tvecs': translation_vectors,
            '_objPoints': obj_points,
        }

    def get_aruco_dict(self) -> cv2.aruco.Dictionary:
        _ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        aruco_dict_type = _ARUCO_DICT.get(self.type, None)
        if aruco_dict_type is None:
            raise ValueError(f"Unknown 'aruco_type' : {self.type}")
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        return aruco_dict

    def get_aruco_params(self, **kwargs) -> cv2.aruco.DetectorParameters:
        # adaptiveThreshConstant: 7.0
        # adaptiveThreshWinSizeMax: 23
        # adaptiveThreshWinSizeMin: 3
        # adaptiveThreshWinSizeStep: 10
        # aprilTagCriticalRad: 0.1745329201221466
        # aprilTagDeglitch: 0
        # aprilTagMaxLineFitMse: 10.0
        # aprilTagMaxNmaxima: 10
        # aprilTagMinClusterPixels: 5
        # aprilTagMinWhiteBlackDiff: 5
        # aprilTagQuadDecimate: 0.0
        # aprilTagQuadSigma: 0.0
        # cornerRefinementMaxIterations: 30
        # cornerRefinementMethod: 0
        # cornerRefinementMinAccuracy: 0.1
        # cornerRefinementWinSize: 5
        # detectInvertedMarker: False
        # errorCorrectionRate: 0.6
        # markerBorderBits: 1
        # maxErroneousBitsInBorderRate: 0.35
        # maxMarkerPerimeterRate: 4.0
        # minCornerDistanceRate: 0.05
        # minDistanceToBorder: 3
        # minMarkerDistanceRate: 0.05
        # minMarkerLengthRatioOriginalImg: 0.0
        # minMarkerPerimeterRate: 0.03
        # minOtsuStdDev: 5.0
        # minSideLengthCanonicalImg: 32
        # perspectiveRemoveIgnoredMarginPerCell: 0.13
        # perspectiveRemovePixelPerCell: 4
        # polygonalApproxAccuracyRate: 0.03
        # useAruco3Detection: False
        params = cv2.aruco.DetectorParameters_create()
        for k, v in kwargs:
            assert hasattr(params, k)
            setattr(params, k, v)
        return params

    @staticmethod
    def get_camera_matrix_and_distortion_coeffs(
            calib_data: dict) -> Tuple[np.ndarray, np.ndarray]:

        intrinsic_mat = np.zeros((3, 3))
        intrinsic_mat[0, 0] = calib_data['color']['intrinsic_mat'][0]
        intrinsic_mat[0, 1] = calib_data['color']['intrinsic_mat'][1]
        intrinsic_mat[0, 2] = calib_data['color']['intrinsic_mat'][2]
        intrinsic_mat[1, 0] = calib_data['color']['intrinsic_mat'][3]
        intrinsic_mat[1, 1] = calib_data['color']['intrinsic_mat'][4]
        intrinsic_mat[1, 2] = calib_data['color']['intrinsic_mat'][5]
        intrinsic_mat[2, 0] = calib_data['color']['intrinsic_mat'][6]
        intrinsic_mat[2, 1] = calib_data['color']['intrinsic_mat'][7]
        intrinsic_mat[2, 2] = calib_data['color']['intrinsic_mat'][8]

        coeffs = np.zeros((4))
        coeffs[0] = calib_data['color']['coeffs'][0]
        coeffs[1] = calib_data['color']['coeffs'][1]
        coeffs[2] = calib_data['color']['coeffs'][2]
        coeffs[3] = calib_data['color']['coeffs'][3]

        return intrinsic_mat, coeffs


if __name__ == '__main__':

    ar_args, remain_args = get_parser().parse_known_args()
    print("========================================")
    print(">>>>> aruco args <<<<<")
    print("========================================")
    for k, v in vars(ar_args).items():
        print(f"{k} : {v}")
    print("========================================")
    if ar_args.ar_only:
        ARW = ArucoWrapper(ar_args)
        ARW.step()
        printout(f"Finished...", 'i')
        exit(1)

    rs_args, _ = get_rs_parser().parse_known_args(remain_args)
    print("========================================")
    print(">>>>> rs args <<<<<")
    print("========================================")
    for k, v in vars(rs_args).items():
        print(f"{k} : {v}")
    print("========================================")

    # 1. Setting up RS
    RSW = RealsenseWrapper(rs_args, rs_args.rs_dev)
    RSW.initialize()
    RSW.set_ir_laser_power(rs_args.rs_laser_power)
    RSW.save_calib()
    RSW.flush_frames(rs_args.rs_fps * 5)

    # 2. Setting up AR
    ARW = ArucoWrapper(ar_args, rsw=RSW)

    # 3. Live loop
    try:
        c = 0
        while True:
            printout(f"Step {c:8d}", 'i')
            frames = RSW.step(
                display=rs_args.rs_display_frame,
                display_and_save_with_key=rs_args.rs_save_with_key
            )
            images, idxs = {}, {}
            for dev_sn, data in frames.items():
                images[dev_sn] = data['color'].reshape(
                    rs_args.rs_image_height, rs_args.rs_image_width, 3)
                idxs[dev_sn] = int(data['timestamp_color'])
            ARW.step(images, idxs)
            if not len(frames) > 0:
                printout(f"Empty...", 'w')
                continue
            c += 1
            if c > rs_args.rs_fps * rs_args.rs_steps:
                break

    except Exception as e:
        printout(f"{e}", 'e')
        printout(f"Stopping RealSense devices...", 'i')
        RSW.stop()

    finally:
        printout(f"Final RealSense devices...", 'i')
        RSW.stop()

    printout(f"Finished...", 'i')
