# METHOD
# go to the main page and save it
# Restrict crawling zone on the site (for speed and not burden website) : Use the search engine provided by the site to get to a page containing the relevant information
# Restricti crawling on the page (for speed) : substract main page to search result page in order to get the +diffs (the difference only in the search result page)
# Browse the difference each time substracting the search result page to the current page
# Give back candidat names for precisie selection
# when precise name available, search for images.

import sys

import urllib2
import lxml.html
import difflib
# TODO a partir de la liste de resultats, trouver le bon.
baseURL = "http://www.casinodrive.fr"
startingURL = baseURL + "/ecommerce/GC-catalog/fr/WE31907/?moderetrait=Z4"

class Crawler:
    def __init__ (self):
        self.main_page = ""
        self.list_of_crawled = set()
        
    def getCandidats (url)
    # Use the RECHERCHE form to reach the target faster
    def searchFor (self, url, target):
        try:
            page = urllib2.urlopen (url)
        except (urllib2.URLError, ValueError):
            return None
        html = page.read()
        page = lxml.html.fromstring (html)
        page.make_links_absolute (baseURL)
        page.forms[0].inputs['query'].value = target
        new_page = lxml.html.submit_form(page.forms[0])
        #print new_page.geturl()
        return new_page
        
        
    # rules for accepting an url, or not
    def accepted (self, url, depth):
        return (depth == 0) or not("ecommerce" in url) or ("layout" in url) or (url in self.list_of_crawled)
        
    def useWebsiteSearch (url, target):
        caca
        
    # parts of the html that have not been seen in a previous page
    def difference (self, html):
        print "Initial size %d" % len(html)
        s_html = html.split('\n')
        s_last_html = html.split('\n')
        diff = difflib.ndiff(s_last_html, s_html)
        for l in diff:
            print l
            print "kikou\n"
        delta = [l for l in diff if l.startswith('+ ')]
        print "Delta size %d" % len(delta)
        return delta
    
    def crawl (self, url, depth = 5):
        if  self.accepted (url, depth):
            #print "Not accepting %s" % url
            return None
        
        self.list_of_crawled.add (url);
        try:
            if not("http" in url):
                url = baseURL + url
            page = urllib2.urlopen (url)
        except (urllib2.URLError, ValueError):
            #print "Except %s" % url
            return None
            
        # retrieve string
        html = page.read();
        interest_html = self.difference (html)
        if ( len (interest_html) == 0 ):
            return None
        self.last_html = interest_html
        
        page = lxml.html.fromstring (interest_html)
        root = {}
        root["children"] = []
        root["url"] = url
        root["content"] = html
        
        print "Level %d: %s" % (depth, url)
        
        for link in page.xpath('//a/@href'):
            child = self.crawl (link, depth - 1)
            if child is not None:
                root["children"].append (child)
        
        return root
    
    
def main(argv=None):
    cricri = Crawler()
    cricri.searchFor (startingURL, 'pago')

if __name__ == "__main__":
    sys.exit(main())
