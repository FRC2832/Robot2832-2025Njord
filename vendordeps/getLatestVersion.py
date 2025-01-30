# This script reads through a vendordep folder and checks what the latest versions are.  This is different from the WpiLib extension, as it only checks what is in
# https://github.com/wpilibsuite/vendor-json-repo, which might not be all the latest versions

import glob
import json
import urllib
import urllib.request
import xml.etree.ElementTree as ET 

# find all vendorDeps
files = glob.glob("*.json")
for fileName in files:
    print(f"\n{fileName}")
    # read the json file
    with open(fileName,'r') as file:
        data = json.load(file)

    #loop through all the maven URLs
    for maven in data['mavenUrls']:
        for depend in data['javaDependencies']:
            # find the parts needed for url, url will always end with /
            url = maven
            if not url.endswith("/"):
                url = url + "/"
            groupId = depend['groupId'].replace(".","/")
            artifactId = depend['artifactId'].replace(".","/")

            # create the maven url
            fullPath = f"{url}{groupId}/{artifactId}/maven-metadata.xml"

            try:
                with urllib.request.urlopen(fullPath) as response:
                    xmlString = response.read()
            except:
                print(f"Error retrieving {fullPath}")
                continue

            tree = ET.fromstring(xmlString)
            latest = tree.find("versioning/latest")
            release = tree.find("versioning/release")
            modify = tree.find("versioning/lastUpdated")
            if depend['version'] != release.text:
                modified = "(Modified) "
            else:
                modified = ""
            print(f"{modified}{depend['artifactId']} Current: {depend['version']} Latest: {latest.text} Release: {release.text} Last Modified: {modify.text}")
