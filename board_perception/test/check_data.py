import os
import json
import cv2
import numpy as np

from board_perception.part_classifier import (BOARD_DATA, LabeledCellExtractor,
                                              reverse_board_state)


def visual_test(directory, name, reverse=False):
    with open(os.path.join(directory, name + '.json')) as b:
        board = json.load(b)
    if reverse:
        board = reverse_board_state(board)
        name += 'R'
    img = cv2.imread(os.path.join(directory, name + '.png'))
    # Create alpha channel
    alpha = .2 * np.ones((img.shape[0], img.shape[1], 1))
    extr = LabeledCellExtractor(alpha, board)
    vert, hori = extr.labeled_cells()
    for ((l, o), c) in vert + hori:
        if l is not None:
            c[:, :, :] = 1.  # Modifies underlying alpha
    img = np.asarray(img * alpha, dtype=np.uint8)
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    for name in ['board_{}'.format(1 + i) for i in range(8)]:
        print(name)
        for r in [False, True]:
            visual_test(BOARD_DATA, name, reverse=r)
