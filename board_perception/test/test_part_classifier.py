from unittest import TestCase
import numpy as np

from board_perception.part_classifier import (
    DATA, H_MARGIN, W_MARGIN, H_CELL, W_CELL, N_ROWS, N_COLUMNS,
    NORTH, SOUTH, EAST, WEST, ROTATION, PART_TAG_LOCATION,
    cell_coordinate, rotate_tag_location, tag_location_from_part,
    inverse_orientation, part_reference_from_tag_location, _reverse_example,
    CellExtractor, LabeledCellExtractor, reverse_board_state, PartDetector)


class DummyImage():

    shape = (600, 400)

    def __getitem__(self, slices):
        return (slices[0].start, slices[0].stop,
                slices[1].start, slices[1].stop)


class DummyImage2(DummyImage):

    def __getitem__(self, slices):
        return np.array([[slices[0].start, slices[0].stop],
                         [slices[1].start, slices[1].stop],
                         ])[:, :, np.newaxis]


class DummyClassifier:

    def fit(self, arrays, labels):
        self.known = {}
        for a, l in zip(arrays, labels):
            self.known[str(a)] = l

    def predict(self, arrays):
        return [self.known[str(a)] for a in arrays]


class DummyDetector(PartDetector):

    def __init__(self):
        self.extr = CellExtractor()
        self.v_classifier = DummyClassifier()
        self.h_classifier = DummyClassifier()


class TestCellCoordinate(TestCase):

    def test_0_0_is_margins(self):
        self.assertEqual(cell_coordinate(0, 0), (H_MARGIN, W_MARGIN))

    def test_0_1(self):
        self.assertEqual(cell_coordinate(0, 1), (H_MARGIN, W_MARGIN + W_CELL))

    def test_1_0(self):
        self.assertEqual(cell_coordinate(1, 0), (H_MARGIN + H_CELL, W_MARGIN))


class TestRotations(TestCase):

    def test_is_rotation(self):
        for (a, b, c, d) in [r.flatten() for r in ROTATION]:
            self.assertEqual(a * d - b * c, 1)

    def test_east_is_identity(self):
        self.assertEqual(tuple(ROTATION[EAST].flatten()), (1, 0, 0, 1))

    def test_rotate_tag_location(self):
        np.testing.assert_array_equal(
            rotate_tag_location((2, 3), EAST), np.array([2, 3]))
        np.testing.assert_array_equal(
            rotate_tag_location((2, 3), WEST), np.array([-2, -3]))
        np.testing.assert_array_equal(
            rotate_tag_location((2, 3), NORTH), np.array([-3, 2]))
        np.testing.assert_array_equal(
            rotate_tag_location((2, 3), SOUTH), np.array([3, -2]))

    def test_rotates_float(self):
        np.testing.assert_array_equal(
            rotate_tag_location((.2, 3.5), SOUTH), np.array([3.5, -.2]))

    def test_tag_location_from_part(self):
        np.testing.assert_array_equal(
            tag_location_from_part('5', np.array([4, 3]), EAST),
            np.array([4, 4.5]))
        np.testing.assert_array_equal(
            tag_location_from_part('5', np.array([4, 3]), SOUTH),
            np.array([5.5, 3]))
        np.testing.assert_array_equal(
            tag_location_from_part('U2', np.array([4, 3]), WEST),
            np.array([3.5, 2]))
        np.testing.assert_array_equal(
            tag_location_from_part('U2', np.array([4, 3]), NORTH),
            np.array([3, 3.5]))

    def test_inverse_orientation(self):
        self.assertEqual(inverse_orientation(NORTH), SOUTH)
        self.assertEqual(inverse_orientation(SOUTH), NORTH)
        self.assertEqual(inverse_orientation(WEST), EAST)
        self.assertEqual(inverse_orientation(EAST), WEST)

    def test_part_reference_from_tag_location(self):
        loc = np.random.randint(10, size=(2,))
        for label in PART_TAG_LOCATION:
            for o in [NORTH, EAST, WEST, SOUTH]:
                np.testing.assert_array_equal(
                    part_reference_from_tag_location(
                        label, tag_location_from_part(label, loc, o), o),
                    loc)


class TestCellExtractor(TestCase):

    def setUp(self):
        self.extr = CellExtractor()
        self.extr.set_image(DummyImage())

    def test_all_peg_indices_len(self):
        self.assertEqual(len(list(self.extr.all_peg_indices())),
                         N_ROWS * N_COLUMNS)
        self.assertEqual(len(list(self.extr.all_peg_indices(False, False))),
                         (N_ROWS - 1) * (N_COLUMNS - 1))
        self.assertEqual(len(list(self.extr.all_peg_indices(last_row=False))),
                         (N_ROWS - 1) * N_COLUMNS)
        self.assertEqual(
            len(list(self.extr.all_peg_indices(last_column=False))),
            N_ROWS * (N_COLUMNS - 1))


class TestWithBoard:

    def setUp(self):
        self.board = {"parts": [
                                {"id": 0,
                                 "label": "4",
                                 "location": [1, 4, "WEST"]
                                 },
                                {"id": 1,
                                 "label": "2",
                                 "location": [3, 1, "NORTH"]
                                 },
                                {"id": 9,
                                 "label": "U2",
                                 "location": [4, 6, "EAST"]
                                 },
                                {  # Last columns
                                 "id": 10,
                                 "label": "5",
                                 "location": [5, 9, "NORTH"]
                                 },
                                {  # Last row
                                 "id": 5,
                                 "label": "S1",
                                 "location": [6, 3, "WEST"]
                                 },
                                ]}


class TestLabeledCellExtractor(TestWithBoard, TestCase):

    def setUp(self):
        super().setUp()
        img = DummyImage()
        self.extr = LabeledCellExtractor(img, self.board)

    def test_set_labels(self):
        labels = {(1, 2.5): ("4", WEST),
                  (2.5, 1): ("2", NORTH),
                  (4.5, 7): ("U2", EAST),
                  (3.5, 9): ("5", NORTH),
                  (6, 1.5): ("S1", WEST)}
        self.assertEqual(labels, self.extr.labels)

    def test_labeled_cells(self):
        v, h = self.extr.labeled_cells()
        self.assertEqual(9 * 7, len(h))
        self.assertEqual(10 * 6, len(v))
        non_empty_h = [(l, o, c) for ((l, o), c) in h if l is not None]
        non_empty_v = [(l, o, c) for ((l, o), c) in v if l is not None]
        self.assertEqual(set([("4", WEST), ("S1", WEST)]),
                         set([(l, o) for (l, o, c) in non_empty_h]))
        self.assertEqual(set([("2", NORTH), ("U2", EAST), ("5", NORTH)]),
                         set([(l, o) for (l, o, c) in non_empty_v]))
        # This is only a non-regression test from a known correct state
        self.assertEqual(set([("4", (106, 148, 111, 129)),
                              ("S1", (537, 579, 71, 89))]),
                         set([(l, c) for (l, o, c) in non_empty_h]))
        self.assertEqual(set([("2", (235, 277, 51, 69)),
                              ("U2", (408, 450, 290, 308)),
                              ("5", (322, 364, 370, 388))]),
                         set([(l, c) for (l, o, c) in non_empty_v]))


class TestReverseBoard(TestWithBoard, TestCase):

    def test_reverse(self):
        reverse = {"parts": [
                             {"id": 0,
                              "label": "4",
                              "location": [5, 5, "EAST"]
                              },
                             {"id": 1,
                              "label": "2",
                              "location": [3, 8, "SOUTH"]
                              },
                             {"id": 9,
                              "label": "U2",
                              "location": [2, 3, "WEST"]
                              },
                             {  # Last columns
                              "id": 10,
                              "label": "5",
                              "location": [1, 0, "SOUTH"]
                              },
                             {  # Last row
                              "id": 5,
                              "label": "S1",
                              "location": [0, 6, "EAST"]
                              },
                             ]}
        self.assertEqual(reverse_board_state(self.board), reverse)


class TestPartDetector(TestWithBoard, TestCase):

    maxDiff = None

    def setUp(self):
        super().setUp()
        self.detector = DummyDetector()
        self.img = DummyImage2()
        extr = LabeledCellExtractor(self.img, self.board)
        vert, hori = extr.labeled_cells()
        vert += [_reverse_example(l_o, c) for (l_o, c) in vert]
        hori += [_reverse_example(l_o, c) for (l_o, c) in hori]
        self.detector.v_classifier.fit(
            np.vstack([c.flatten() for l, c in vert]),
            [l for l, c in vert])
        self.detector.h_classifier.fit(
            np.vstack([c.flatten() for l, c in hori]),
            [l for l, c in hori])

    def test_analyse_board(self):
        found_board = self.detector.analyse_board(self.img)
        found = set([(p['label'], tuple(p['location']))
                     for p in found_board['parts']])
        real = set([(p['label'], tuple(p['location']))
                    for p in self.board['parts']])
        self.assertEqual(found, real)


def visual_test():
    import os
    import json
    import cv2
    data = os.path.join(DATA, 'boards')
    name = 'board_evaluation'
    with open(os.path.join(data, name + '.json')) as b:
        board = json.load(b)
    img = cv2.imread(os.path.join(data, name + '.jpg'))
    # Create alpha channel
    alpha = .2 * np.ones((img.shape[0], img.shape[1], 1))
    extr = LabeledCellExtractor(alpha, board)
    vert, hori = extr.labeled_cells()
    for ((l, o), c) in vert + hori:
        if l is not None:
            c[:, :, :] = 1.  # Modifies underlying alpha
    img = np.asarray(img * alpha, dtype=np.uint8)
    cv2.imshow('tags', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    visual_test()
