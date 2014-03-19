import urllib2
import difflib

url1 = "http://www.casinodrive.fr/ecommerce/GC-catalog/fr/WE31907/?moderetrait=Z4"
url2 = "http://www.casinodrive.fr/ecommerce/affichageDetailProduit/WE31907/F-47104-jus-de-fruits/P-52703-alvalle-gazpacho-soupe-froide"

def IS_CHAR_JUNK (s):
    if s in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
        return True;
    return False;
    
print "Open url1"
page = urllib2.urlopen (url1)
s1 = page.read();
print "Open url2"
page = urllib2.urlopen (url2)
s2 = page.read();

l1 = s1.split('\n');
l2 = s2.split('\n');
print "Size of urls %d %d" % (len(l1), len(l2))
print "Diffing..."
diff = difflib.ndiff (l1, l2)#, None, IS_CHAR_JUNK)

delta_plus = [l for l in diff if l.startswith('+ ')]

print len(delta_plus)
for l in delta_plus:
    print l
