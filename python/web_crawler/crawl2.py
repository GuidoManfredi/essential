# METHOD
# go to the main page and save it
# Restrict crawling zone on the site (for speed and not burden website) : Use the search engine provided by the site to get to a page containing the relevant information (also allow less burden on the site)
# Restricti crawling on the page (for speed) : substract main page to search result page in order to get the +diffs (the difference only in the search result page)
# Browse the difference each time substracting the search result page to the current page
# Give back candidat names for precise selection
# when precise name available, search for images.
# !!! Be carefull, some products appear in the main page and thus may be cleaned up later in the interest_page !!!
# !!! Problem if there is no match for the querried word in the search form !!!
import sys

import urllib2
import lxml.html
import difflib
import cStringIO
import Image
# TODO proposer une liste de candidats (nom + enventuellement image) et laisser choisir l'utilisateur
baseURL = "http://www.casinodrive.fr"
startingURL = baseURL + "/ecommerce/GC-catalog/fr/WE31907/?moderetrait=Z4"

class Crawler:
    def __init__ (self):
        self.main_s = ""
        self.list_of_crawled = set()
        
    def getImages (self, url, target, folder):
        print 'Looking for object ' + target
        main_page = urllib2.urlopen (url);
        self.main_s = main_page.read()
        interest_s = self.useWebsiteSearch(url, target)
        cleaned_interest_s = self.cleanup(interest_s)
        (names, links) = self.getCandidats(cleaned_interest_s)
        if (len(names) != 0):
            print 'Found ' + str(len(names)) + ' correspondences.'
            #candidate = selectCandidate(candidats)
            print 'Retrieving images for ' + names[0]
            self.retrieveImages (names[0], links[0], folder)
        else:
            print 'Couldn\'t find any ' + target + ' object.'
        
    # Use the RECHERCHE form to reach the target faster
    def useWebsiteSearch (self, url, target):
        try:
            page = urllib2.urlopen (url)
        except (urllib2.URLError, ValueError):
            return None
        s = page.read()
        html = lxml.html.fromstring (s)
        html.make_links_absolute (baseURL)
        html.forms[0].inputs['query'].value = target
        new_page = lxml.html.submit_form(html.forms[0])
        new_s = new_page.read()
        return new_s

    # keep only parts of the interest page that are not on the main page
    def cleanup (self, s):
        seq = s.split('\n')
        seq_main = self.main_s.split('\n')
        diff = difflib.ndiff(seq_main, seq)
        delta = [l for l in diff if l.startswith('+ ')]
        delta = "".join(delta)
        return delta

    # Return a list of candidat names and their associated urls
    def getCandidats (self, s): 
        html = lxml.html.fromstring (s)
        html.make_links_absolute (baseURL)
        #lxml.html.open_in_browser(html)
        names = list()
        links = list()
        for name in html.xpath('//@alt'):
            names.append(name)
            
        for a in html.xpath('//a'):
            if a.attrib.get('title') in names:
                links.append (a.attrib.get('href'))
        return (names, links)
    
    def retrieveImages (self, name, url, folder):
        print "Retrieving images for " + name
        page = urllib2.urlopen (url);
        s = page.read()
        cleaned_s = self.cleanup(s)
        
        html = lxml.html.fromstring (cleaned_s)
        #lxml.html.open_in_browser(html)

        i = 0
        links = set()
        for a in html.xpath('//a'):
            title = a.attrib.get('title')
            if (type(title) is str or type(title) is unicode):
                if (title in name):           
                    url_img = a.attrib.get('href')
                    links.add (url_img);
        
        for link in links:
            file_name = folder + name + '_' + str(i) + '.jpg'
            self.saveImage (link, file_name)
            i += 1
                
    def saveImage (self, url, file_name):
        page = cStringIO.StringIO(urllib2.urlopen(url).read())
        img = Image.open(page)
        if img.size == (800, 800):
            print "Saving " + file_name
            img.save(file_name)

def main(argv=None):
    cricri = Crawler()
    cricri.getImages (startingURL, 'faezfa', "./images/")

if __name__ == "__main__":
    sys.exit(main())
