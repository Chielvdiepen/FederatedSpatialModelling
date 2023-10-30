from mymath import rssi_to_distance

class Edge:

    def __init__(self, src, dst, rssi):
        self.id = (src.uuid,dst.uuid)
        self.src = src
        self.dst = dst
        self.rssi = rssi
        self.dist = rssi_to_distance(rssi)

    def __str__(self) -> str:
        return f"({self.src.uuid},{self.dst.uuid},{self.dist:.3f})"
    
    def __repr__(self):
        return str(self)
    
    def __hash__(self) -> int:
        return hash(self.id)
    
    def __eq__(self, other: object) -> bool:
        return self.__hash__() == other.__hash__()
    
    def compare(self, other: object):
        return (self.src.uuid, self.dst.uuid) == (other.src.uuid, other.dst.uuid) or (self.src.uuid, self.dst.uuid) == (other.dst.uuid, other.src.uuid)