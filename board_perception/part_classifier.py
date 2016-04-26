import numpy as np

N_ROWS = 7
N_COLUMNS = 10

# Board dimensions, in mm
H_MARGIN = 13.5  # Margins from first peg to detected board border
W_MARGIN = 14.
H_CELL = 168. / (N_ROWS - 1)  # Cell dimensions from board dimensions
W_CELL = 251.5 / (N_COLUMNS - 1)
H = H_CELL * (N_ROWS - 1) + 2 * H_MARGIN  # Total board dimensions
W = W_CELL * (N_COLUMNS - 1) + 2 * W_MARGIN

ORIENTATIONS = {'NORTH': 0, 'EAST': 1, 'SOUTH': 2, 'WEST': 3}
globals().update(ORIENTATIONS)  # Define NORTH, EAST, ... as global variables
INVERSE_ORIENTATIONS = {'NORTH': 'SOUTH', 'EAST': 'WEST',
                        'SOUTH': 'NORTH', 'WEST': 'EAST'}

ROTATION = [(0, -1, 1, 0), (1, 0, 0, 1), (0, 1, -1, 0), (-1, 0, 0, -1)]
ROTATION = [np.array(r).reshape((2, 2)) for r in ROTATION]
# Location of the recognizable tag on parts.
# Expressed as shift from reference peg (generally top left in upright
# position, that is EAST orientation).
PART_TAG_LOCATION = {
    '2': np.array([0, 0.5]),
    '3': np.array([0, 0.5]),
    '4': np.array([0, 1.5]),
    '5': np.array([0, 1.5]),
    '6': np.array([0, 2.5]),
    's1': np.array([0, 1.5]),
    's2': np.array([0, 1.5]),
    'd1': np.array([0, 1.5]),
    'u2': np.array([0.5, 1]),
    }


def cell_coordinate(row, column):
    # Cell coordinate in mm
    return (H_MARGIN + H_CELL * row, W_MARGIN + W_CELL * column)


def rotate_tag_location(loc, orientation):
    """Tag location from part reference point according to part orientation.
    """
    loc = np.asarray(loc)
    return ROTATION[orientation].dot(loc)


def tag_location_from_part(label, location, orientation):
    diff = rotate_tag_location(PART_TAG_LOCATION[label], orientation)
    return location + diff


def inverse_orientation(orientation):
    return (orientation + 2) % 4


def part_reference_from_tag_location(label, loc, orientation):
    return tag_location_from_part(label, loc, inverse_orientation(orientation))


def tag_from_part_coordinate(loc, label):
    """Returns the location of the part from its tag and label.
    """
    row, col, ori = loc
    drow, dcol, segment = PART_TAG_LOCATION[label]


def reverse_cell_location_triplet(loc):
    """Rotate part location by pi in the board."""
    r, c, o = loc
    r, c = ((N_ROWS - 1) - r, (N_COLUMNS - 1) - c)
    return [r, c, INVERSE_ORIENTATIONS[o.upper()]]


def reverse_board_state(board):
    """Rotate parts location by pi in the board."""
    rev = deepcopy(board)
    for p in rev["parts"]:
        p["location"] = reverse_cell_location_triplet(p["location"])
    return rev


class CellExtractor:

    def set_image(self, img):
        self.img = img
        steps_real = (H_CELL / 4., W_CELL / 4.)
        self.dh, self.dw = self.image_coordinate(steps_real)

    @property
    def width(self):
        return self.img.shape[0]

    @property
    def height(self):
        return self.img.shape[1]

    def image_coordinate(self, coordinates):
        # Converts coordinates in mm to pixels
        i, j = coordinates
        return (int(i * self.width / H), int(j * self.height / W))

    def cell_image(self, row, column):
        i, j = self.image_coordinate(cell_coordinate(row, column))
        return self.img[i - self.dh:i + self.dh, j - self.dw:j + self.dw, :]

    def all_peg_indices(self, last_row=True, last_column=True):
        for row in range(N_ROWS - (not last_row)):
            for column in range(N_COLUMNS - (not last_column)):
                yield (row, column)

    def all_horizontal_cells(self):
        for row, column in self.all_peg_indices(last_column=False):
            yield (row, column + .5, self.cell_image(row, column + .5))

    def all_vertical_cells(self):
        for row, column in self.all_peg_indices(last_row=False):
            yield (row + .5, column, self.cell_image(row + .5, column))


class LabeledCellExtractor:
    """Extracts cells with labels for training.
    """

    def __init__(self, img, board_state):
        self.cell_extr = CellExtractor()
        self.cell_extr.set_image(img)
        self.labels = {}
        self.set_labels(board_state)

    def set_labels(self, board_state):
        """Populates self.labels from board state.

           self.labels[(row, column)] = (label, orientation)
        """
        for part in board_state['parts']:
            label = part['label']
            part_loc = part['location']
            orientation = ORIENTATIONS[part_loc[2].upper()]
            tag_loc = tag_location_from_part(label, part_loc[:2], orientation)
            self.labels[(tag_loc[0], tag_loc[1])] = (label, orientation)

    def _to_label_and_cell(self, row, column, cell):
        if (row, column) in self.labels:
            return (self.labels[(row, column)], cell)
        else:
            return ((None, None), cell)

    def _to_labels_and_cells(self, cells):
        return [self._to_label_and_cell(row, col, cell)
                for (row, col, cell) in cells]

    def labeled_cells(self):
        """Returns lists of horizontal and vertical cells with labels.

        Labels are (label, orientation) and (Non, None) for non-tag cells.
        """
        horizontal = self._to_labels_and_cells(
            self.cell_extr.all_horizontal_cells())
        vertical = self._to_labels_and_cells(
            self.cell_extr.all_vertical_cells())
        return vertical, horizontal
