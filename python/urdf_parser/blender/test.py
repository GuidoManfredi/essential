import bpy

Obj = bpy.data.objects["Cube"]
Mesh = bpy.data.meshes["Cube"]
Mesh.vertices[0].co[0] = 2
Mesh.edges[0].vertices

Camera = bpy.data.objects["Camera"]
Camera.location.x = 1


