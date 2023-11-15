from triangle import Triangle

class Tetrahedon:

    def __init__(self, triangle1: Triangle, triangle2: Triangle, triangle3: Triangle):
            self.triangles = [triangle1,triangle2,triangle3]
            self.nodes = [triangle1.nodes[0]] + list(set(triangle1.nodes + triangle2.nodes + triangle3.nodes) - {triangle1.nodes[0]})          
            self.unique = sorted(set(node.uuid for node in self.nodes))
            self.ID = "|".join(map(str, self.unique))

    def __str__(self) -> str:
        triangle_str = ', '.join(map(str, [triangle.ID for triangle in self.triangles]))
        node_str = ', '.join(map(str, [node.uuid for node in self.nodes]))
        return f"Tetrahedon({self.ID}): triangles=({triangle_str}), Nodes=({node_str})"
    
    def __repr__(self):
        return str(self)