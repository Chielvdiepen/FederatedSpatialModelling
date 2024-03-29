from triangle import Triangle

class Adjacent:

    def __init__(self, triangle: Triangle):
        self.triangle = triangle
        self.mapAngle = 0
        self.otherCoord = (None,0)
        self.basealtiX = (None,None)
        self.basealtiH = (None,None)
        self.mappedHeight = None

    def __str__(self) -> str:
        if self.mappedHeight == None:
            return f"Triangle({self.triangle.ID}): base-coord=({self.otherCoord[0]},{self.otherCoord[1]:.3f}), map angle= {self.mapAngle:.3f} | xpos altitude=({self.basealtiX[0]},{self.basealtiX[1]:.3f}) | altitude height=({self.basealtiH[0]},{self.basealtiH[1]:.3f})-> Mapped Height = {self.mappedHeight}"
        else:
            return f"Triangle({self.triangle.ID}): base-coord=({self.otherCoord[0]},{self.otherCoord[1]:.3f}), map angle= {self.mapAngle:.3f} | xpos altitude=({self.basealtiX[0]},{self.basealtiX[1]:.3f}) | altitude height=({self.basealtiH[0]},{self.basealtiH[1]:.3f})-> Mapped Height = ({self.mappedHeight[0]:.3f},{self.mappedHeight[1]:.3f},{self.mappedHeight[2]:.3f})"
    
    def __repr__(self):
        return str(self)
    
    def setCoord(self, coord):
        self.otherCoord = coord
    
    def setAngle(self, angle):
        self.mapAngle = angle

    def setAltiX(self, xpos):
        self.basealtiX = xpos
    
    def setAltiH(self, height):
        self.basealtiH = height

    def setMapHeight(self, height):
        self.mappedHeight = height