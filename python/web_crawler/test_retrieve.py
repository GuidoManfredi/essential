import urllib2
import cStringIO
import Image

url = "http://imgart.casinodrive.fr/imageserver/MC2/0/646590_M1_S1.jpg"

page = cStringIO.StringIO(urllib2.urlopen(url).read())
img = Image.open(page)
print "%d %d" % img.size
img.save('out.jpg')

alt = image.attrib.get('alt')
            if not(alt is None) and (alt in name):
                url_img = image.attrib.get('src')
                print url_img
                page = cStringIO.StringIO(urllib2.urlopen(url_img).read())
                img = Image.open(page)
                if img.size == (800, 800):
                    img.save(folder + alt + '_' + str(i) + '.jpg')
                    ++i
