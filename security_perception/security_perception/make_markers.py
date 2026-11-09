"""Generate printable ArUco marker images.

    ros2 run security_perception make_markers --ids 3 5 7 --out ~/markers

Prints one PNG per ID (with a white border and the ID written underneath).
Print them so the black square is exactly the `marker_size` set in
config/perception.yaml (default 10 cm), then measure it with a ruler.
"""
import argparse
import os

import cv2
import numpy as np


def make_marker(dictionary, marker_id, side_px):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ids', type=int, nargs='+', default=[3, 5, 7])
    parser.add_argument('--dict', default='DICT_4X4_50')
    parser.add_argument('--px', type=int, default=600, help='marker side in pixels')
    parser.add_argument('--out', default='~/markers')
    args = parser.parse_args()

    out = os.path.expanduser(args.out)
    os.makedirs(out, exist_ok=True)
    dict_id = getattr(cv2.aruco, args.dict)
    dictionary = (cv2.aruco.getPredefinedDictionary(dict_id)
                  if hasattr(cv2.aruco, 'getPredefinedDictionary')
                  else cv2.aruco.Dictionary_get(dict_id))
    border = args.px // 6
    for marker_id in args.ids:
        m = make_marker(dictionary, marker_id, args.px)
        page = np.full((args.px + 3 * border, args.px + 2 * border), 255, np.uint8)
        page[border:border + args.px, border:border + args.px] = m
        cv2.putText(page, f'ID {marker_id}  ({args.dict})', (border, args.px + 2 * border),
                    cv2.FONT_HERSHEY_SIMPLEX, args.px / 500, 0, 2, cv2.LINE_AA)
        path = os.path.join(out, f'aruco_{args.dict}_{marker_id}.png')
        cv2.imwrite(path, page)
        print(path)


if __name__ == '__main__':
    main()
