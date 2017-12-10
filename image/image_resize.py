import os
import cv2

# Create a list of files from the current directory who's last 4 characters
# as lowercase are either '.jpg' or '.png'
files = [ f for f in os.listdir('.') if f[-4:].lower() in ('.jpg','.png') ]

DRYRUN=False

for (index,filename) in enumerate(files):
  extension = os.path.splitext(filename)[1]
  newname = "%02d%s" % (index,extension)
  # if os.path.exists(newname):
  #   print "Cannot rename %s to %s, already exists" % (filename,newname)
  #   continue
  if DRYRUN:
    print "Would rename %s to %s" % (filename,newname)
  else:
    print "Resizing %s to %s" % (filename,newname)
    image = cv2.imread(filename)
    resized_image = cv2.resize(image, (55,55))
    cv2.imwrite(newname, resized_image)