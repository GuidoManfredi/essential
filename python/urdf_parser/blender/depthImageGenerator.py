#!/usr/bin/env python

import bpy

def loadFilenames(rootFile):

	# Open file containing name of all models
	f = open(rootFile, 'r');

	# load file with objects filenames
	filenames = f.readlines()

	# close rootFile
	f.close()

	return filenames

# object preprocessing
def preprocessing(Object):
	# move object to origin
	Object.location = (0, 0, -Object.dimensions.z/2)

	return

def main():
	# get handle on camera
	Camera = bpy.data.objects["Camera"]

	# import dodecahedron : 5 vertices per face, 20 total
	bpy.ops.wm.collada_import(filepath="/home/gmanfred/devel/scripts/blender/dodecahedron.dae")
	# select dodecahedron object
	Dodec = bpy.data.objects["SketchUp"]
	Dodec.location = (0, 0, -Dodec.dimensions.z/2)
	# FIXME name mesh as object (correlate both object and mesh names)
	# take handle on dodecahedron mesh
	DodecMesh = bpy.data.meshes["ID"]
	# do not render the dodecahedron
	Dodec.hide_render)

	# Open file containing name of all models and load them in a vector
	filenames = loadFilenames("/home/gmanfred/devel/scripts/blender/rootfile.txt")

	# for each object
	for o in range(len(filenames))
		# import .3DS object, don't forget to strip \n
		# from end of filename
		bpy.ops.import_scene.autodesk_3ds(filenames[o].rstrip)
		# get handle on object
		Object = bpy.data.objects[filenames[o].rstrip]
		# object preprocessing
		preprocessing(Object);

		# for each dodecahedron size (10 sizes)
		for s in range(1,10)
			# FXME resize dodecahedron (factor of k)
			scale = 0.5*s
			Dodec.scale = (scale, scale, scale)
			# center on origin. Origin of model is center of bottom face
			Dodec.location = (0, 0, -Dodec.dimensions.z/2)
			# for each position on dodecahedron
			for v in range(len(DodecMesh.vertices))
				# FIXME doesn't change position when mesh scaled
				# position camera at dodecahedron vertex
				Camera.location = DodecMesh.vertices[v].co
				# render image
				bpy.ops.render.render()
				#bpy.ops.image.save_as(filename)
				# specify save filename
				filename = "/home/gmanfred/devel/scripts/blender/object" + str(o) + "scale" + str(s) + "depth" + str(v) + ".bmp"
				# FIXME contexte incorrect
				bpy.ops.image.save_as(filepath=filename)
			# end		
		# end

		# remove object
		bpy.ops.object.select(Object.name)
		bpy.ops.object.delete()
	# end
	return

if __name__ == "__main__":
    main()
