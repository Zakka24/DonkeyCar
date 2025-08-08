#!python3

from PIL import Image
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.affinity import rotate
import matplotlib.pyplot as plt
import os

### DEBUG ###
SHOW_TRACK: bool = False #on true, an immage of the track will be saved
TRACK_IMG: str = os.path.expanduser('~/mycar/track_plot/track')

RATIO: float = 1.0
X_TRANSLATION: float = 0.0 
Y_TRANSLATION: float = 0.0
STRAIGHTS: float = 1.75
WIDTH: float = 2.5
ROTATE_TRACK = 0	#angle [deg] with which the track is rotated

class PathToBeFollowed:
    '''
    This class simply creates a track as a set of points.
    This class can be used together with interpolation to calculate
    the distance and angle error of the car.
    '''

    def __init__(self):
        track_center = Polygon([
            (-WIDTH/2 * RATIO + X_TRANSLATION, STRAIGHTS/2 * RATIO + Y_TRANSLATION), 
            (WIDTH/2 * RATIO + X_TRANSLATION, STRAIGHTS/2 * RATIO + Y_TRANSLATION), 
            (WIDTH/2 * RATIO + X_TRANSLATION, -STRAIGHTS/2 * RATIO + Y_TRANSLATION), 
            (-WIDTH/2 * RATIO + X_TRANSLATION, -STRAIGHTS/2 * RATIO + Y_TRANSLATION), 
        ])
        
        track_top_curve = Point(0 + X_TRANSLATION, STRAIGHTS/2*RATIO + Y_TRANSLATION)\
        .buffer(WIDTH/2 * RATIO ,quad_segs=100)
        
        track_bottom_curve =\
        Point(0 + X_TRANSLATION, -STRAIGHTS/2*RATIO + Y_TRANSLATION)\
        .buffer(WIDTH/2 * RATIO,quad_segs=100)

        self.track = unary_union([track_center, track_top_curve, track_bottom_curve]) # Indianapolis like race track
        self.track = rotate(self.track, ROTATE_TRACK, origin='center', use_radians=False)


        if SHOW_TRACK:
            #save the track as an image
            plt.figure(figsize=(4, 6))
            plt.plot(*self.track.exterior.xy)
            #plt.show()
            plt.savefig(TRACK_IMG + '.png')
            
            image = Image.open(f"{TRACK_IMG}.png")
            rotated_image = image.rotate(-90, expand=True)
            rotated_image.save(TRACK_IMG + "_view.png")
            image.close()

    def get_track(self):
        return self.track

def main():
    PathToBeFollowed()

if __name__ == "__main__":
    main()   

