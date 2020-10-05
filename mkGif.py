import glob
from PIL import Image
import imageio
import os
path='./gifPics/'
gifTarget="./anim.gif"
images=[]
for f in os.listdir(path):
	fPath=os.path.join(path,f)
	images.append(imageio.imread(fPath))
#images=sorted(images.all())
imageio.mimsave(gifTarget,images,duration=1,fps=20)
#l=len(images)
#img,*imgs=[Image.open(f) for f in sorted(glob.glob(path+'/*'))]
#images[0].save(gifTarget,save_all=True,append_images=images[1:],optimize=False,duration=19,loop=0)
#image_folder=os.fsencode(path)

#filenames=[]
#
#for file in os.listDIr(image_folder):
#	filename=os.fsdecode(file)
#	if filename.endswith('.jpeg','.png'.'.gif'):
#		filenames.append(filename)
#filenames.sort()
#images = list(map(lambda filename: imageio.imread(filename), filenames))
#
#imageio.mimsave(os.path.join('my_very_own_gif.gif'), images, duration = 0.04)
