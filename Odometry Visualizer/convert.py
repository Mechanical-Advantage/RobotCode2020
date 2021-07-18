import jsmin
import cssmin
import htmlmin
import re
import base64
import sys

input_html = open("src/index.html", "r")
output_html = open("Visualize.html", "w")


# Reads a generic text file
class TextProcessor():
    def process(self, input):
        return open(input, "r").read()


# Reads a minified JavaScript text file
class JSProcessor():
    def process(self, input):
        return jsmin.jsmin(open(input, "r").read())


# Reads a minified CSS text file
class CSSProcessor():
    def process(self, input):
        return cssmin.cssmin(open(input, "r").read())


# Reads an image file as base64
class ImgProcessor():
    def __init__(self, type):
        self.type = type

    def process(self, input):
        return "data:image/" + self.type + ";base64," + base64.b64encode(open(input, "rb").read()).decode("UTF-8")


# Lookup of appropriate processor
processors = {
    "js": JSProcessor(),
    "css": CSSProcessor(),
    "png": ImgProcessor("png"),
    "jpg": ImgProcessor("jpeg"),
    "jpeg": ImgProcessor("jpeg")
}

# Process input HTML
html = input_html.read()
input_html.close()

# Find all files
filenames = []
index = 0
while index < len(html):
    index = html.find("$(", index)
    if index == -1:
        break
    index += 2
    end_index = html[index:].find(")")
    filenames.append(html[index:index + end_index])

# Replace all files
for filename in filenames:
    extension = filename.split(".")[-1]
    if extension in processors.keys():
        output = processors[extension].process("src/" + filename)
    else:
        output = TextProcessor().process("src/" + filename)
    html = html.replace("$(" + filename + ")", output)

# Save to output file
html = htmlmin.minify(html)
output_html.write(html)
output_html.close()
