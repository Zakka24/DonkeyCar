#!python3

from PIL import Image
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
import matplotlib.pyplot as plt

### DEBUG ###
SHOW_TRACK: bool = True #on true, an immage of the track will be saved
TRACK_IMG: str = 'track'

class PathToBeFollowed:
    '''
    This class simply creates a track as a set of points.
    This class can be used together with interpolation to calculate
    the distance and angle error of the car.
    '''

    def __init__(self):
        track_center = Polygon([(-1, 1), (1, 1), (1, -1), (-1, -1)])
        track_top_curve = Point(0, 1).buffer(1,quad_segs=360)
        track_bottom_curve = Point(0, -1).buffer(1,quad_segs=360)

        self.track = unary_union([track_center, track_top_curve, track_bottom_curve]) # Indianapolis like race track

        if SHOW_TRACK:
            #save the track as an image
            plt.figure(figsize=(4, 6))
            plt.plot(*self.track.exterior.xy)
            plt.savefig(TRACK_IMG + '.png')

    def get_track(self):
        return self.track

def main():
    PathToBeFollowed()

if __name__ == "__main__":
    main()   